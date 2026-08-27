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

  uint16_t state_201 = 3;  // 1 = standby, 3 = active
};

#endif
