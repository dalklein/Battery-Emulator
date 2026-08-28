#ifndef LG_RESU_PRIME_MODBUS_H
#define LG_RESU_PRIME_MODBUS_H

#include "../devboard/utils/types.h"
#include "ModbusInverterProtocol.h"

/* Emulates an LG RESU10H Prime toward a Delta E-series inverter (E4/E6/E8/E10-TL-US).
 *
 * The Delta is the Modbus RTU master; the battery answers on server ID 15 (0x0F).
 * A second battery may live at 0x0E -- a single-battery install leaves it unanswered
 * forever and the inverter does not mind.
 *
 * There is no vendor register map. Everything here is reverse-engineered from live
 * traffic between a Delta E6-TL-US and a real RESU10H Prime, cross-checked against
 * SolarEdge captures of the same battery, the vendor apps, and LG's datasheets.
 * Confidence is noted per register in the .cpp; treat anything marked "unknown" as
 * a value to reproduce rather than a meaning to rely on.
 *
 * The read contract is 22 fn3 patterns covering 70 registers. The emulator must
 * answer the UNION of what any inverter asks for -- SolarEdge reads 101-142
 * contiguously and uses fn4, the Delta cherry-picks with fn3 and additionally reads
 * 234 and 2011-2034. A sparse map returning 0 for anything unpopulated is silently
 * wrong rather than an error.
 */
class LgResuPrimeModbusInverter : public ModbusInverterProtocol {
 public:
  LgResuPrimeModbusInverter() : ModbusInverterProtocol(kServerId) {}

  const char* name() override { return Name; }
  bool setup() override;
  void update_values() override;

  static constexpr const char* Name = "LG RESU10H Prime over Modbus RTU (Delta E-series)";
  static constexpr int kServerId = 15;  // 0x0F -- Primary. 0x0E is an optional second pack.

 private:
  void publish_identity();   // static block: IDs, firmware versions, ASCII serial
  void publish_telemetry();  // the live 200-block
  void publish_limits();     // charge/discharge limits, including the derived 226
  void publish_alarms();     // 2001-2034, all zero until we ever capture a fault

  // 1101 is the write side of read-register 201: the Delta writes 1101 and 201 follows.
  // Proven passively 2026-08-26 -- the Delta writes 1101<-1 about 3 s before it stops
  // polling (a graceful shutdown command) and 201 reads 1 roughly 0.7 s later.
  void mirror_config_writes();

  uint16_t state_201 = 1;   // 1 = standby, 3 = active. Starts in standby: the real
                            // pack cannot be active before the inverter enables it.
  bool standby_commanded = false;  // latched by 1101 <- 1, released by 1101 <- 3

 public:
  /* Synthesising the pack side when there is no DC-DC.
   *
   * A real RESU reports BOTH sides of its internal converter: 202/220 are the
   * inverter-facing bus (~410 V) and 215/216 the pack behind it (~170 V). A
   * Battery-Emulator install has no converter, so without help 202 == 215 and the
   * emulator does not look like a real pack.
   *
   * The simplest faithful fix is to rescale by SERIES CELL COUNT, which is really just
   * "report the same volts-per-cell, with the RESU's cell count":
   *
   *     215 = 202 * kResuCellsInSeries / cells_in_series
   *     216 = 220 * cells_in_series / kResuCellsInSeries
   *
   * Power is preserved exactly, so 203 == 215*216/100 and 203 == 202*220/100 both still
   * hold, and 226 = 100*213/215 lands correctly. For a 96S Nissan Leaf at 350-400 V this
   * yields 153-175 V, essentially the 147-174 V the real RESU was measured at -- not a
   * coincidence, since a 3.9 V Leaf cell is a 3.9 V RESU cell.
   *
   * Deliberately LOSSLESS. Modelling DC-DC efficiency here would be less faithful, not
   * more: the real RESU reports both sides reproducing 203 to about 1 %, i.e. it publishes
   * them as if lossless. Its real losses appear only in the cumulative energy counters.
   *
   * Set to 0 to disable and report the bus voltage on both sides (the default, correct
   * when a real DC-DC-equipped pack sits behind the emulator).
   */
  uint8_t cells_in_series = 0;
  static constexpr uint8_t kResuCellsInSeries = 42;  // 2 modules x 21, 1P, NMC

  /* MIRROR MODE -- for milestones 2 and 3, where a REAL RESU is still on DC.
   *
   * Another device (iot-rpi) masters the real battery on its own segment and republishes
   * every register to MQTT. With mirror mode on, this emulator serves those values
   * verbatim instead of deriving them from the datalayer, so the inverter sees exactly
   * what the real pack said -- including registers that have no datalayer equivalent
   * (222-224, 228, 230, 234, 141, 142, the 32-bit energy pairs).
   *
   * Leave it OFF for milestones 4 and 5, where a Leaf pack is the real thing and the
   * datalayer is the source of truth.
   *
   * Topic form: <prefix>/lg_<reg>/state, e.g. lg/master/sensor/lg_201/state
   */
  bool enable_mirror(const char* topic_filter);
  // Matches the default of pi_rs485_sniffer/scripts/lg_master.py
  static constexpr const char* kDefaultMirrorTopic = "lg/master/sensor/+/state";
  bool mirror_enabled() const { return mirror_on; }
  // Milestones 4-5: a Leaf pack is the real thing and the datalayer is the truth.
  // The MQTT subscription stays registered but its data is ignored.
  void disable_mirror() { mirror_on = false; }
  bool mirror_is_fresh() const;

  /* How long mirrored data may go unrefreshed before the emulator stops asking the
   * inverter to move power. The Pi republishes on change plus a 5 s heartbeat, so a gap
   * this long means the link, the broker, or the master is gone -- not a hiccup.
   *
   * 5 minutes, not 15 s (user, 2026-08-28): the degraded state below stops the house
   * discharging, and a WiFi blip or a broker restart must not do that. 60 consecutive
   * heartbeats have to go missing first.
   */
  static constexpr uint32_t kMirrorStaleMs = 300000;

  /* SOC reported once the mirror goes stale, in 0.1 % -- 10.1 % (user, 2026-08-28).
   *
   * The one lever the Delta is PROVEN to obey. Measured 2026-08-26: the discharge floor
   * is enforced inverter-side from the SOC the battery reports, as a hard step to 0 W in
   * one poll interval, about a point above the Backup SoC setting -- observed cutting at
   * 11 % with Backup SoC at 10 %.
   *
   * 10.1 % is below that cut yet above the pack's own 8 % Protection Limit, so the
   * BATTERY is never the one asked to fault. It is also a value no real pack reports to a
   * tenth, so it is a fingerprint in any later capture. The top end needs no equivalent:
   * the zeroed limits already stopped charging when observed falling at full SOC.
   */
  static constexpr uint16_t kStaleSocTenthPct = 101;

 private:
  bool mirror_on = false;
  void apply_mirror();
};

#endif
