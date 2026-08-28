#include "LG-RESU-PRIME-MODBUS.h"

#include "../datalayer/datalayer.h"
#include "../communication/rs485/comm_rs485.h"
#include "../devboard/hal/hal.h"
#include "../devboard/mqtt/mqtt.h"
#include "../devboard/utils/enable_sense.h"
#include "../battery/Battery.h"
#include "../lib/eModbus-eModbus/RTUutils.h"
#include "../devboard/utils/logging.h"

#include <stdlib.h>
#include <string.h>

/* Register meanings are inference fitted to observed traffic between a Delta E6-TL-US
 * and a real LG RESU10H Prime. There is no vendor register map. Confidence per register:
 *
 *   proven   reproduces physics or matches a vendor app / datasheet exactly
 *   high     consistent across two sites and multiple captures
 *   unknown  reproduce the value; do not rely on the meaning
 */

namespace {

// ---------------------------------------------------------------------------
// Identity block -- SITE SPECIFIC.
//
// These are the values of the unit this was reverse-engineered against. They are
// what an inverter uses to recognise the pack, so an emulator must present a
// plausible, self-consistent set. TODO: make configurable (settings/NVS) rather
// than compiled in, so a second install does not need a rebuild.
// ---------------------------------------------------------------------------
constexpr uint16_t kBatteryIdHi = 32229;  // 102 -- shown by the Delta app as "Battery1 SN"
constexpr uint16_t kBatteryIdLo = 62317;  // 103    (0x7DE5 0xF36D)

// 110/111 = DC-DC firmware, 112/113 = BMS firmware, one byte per version field.
// LG's compatibility list pins Delta E-series to BMS V2.5.4.4 / DC-DC V1.4.0.0,
// and reports EXACT versions rather than minimums -- so these are not free choices.
constexpr uint16_t kDcDcFwHi = 0x0104;  // 110 -> 1.4
constexpr uint16_t kDcDcFwLo = 0x0000;  // 111 -> 0.0
constexpr uint16_t kBmsFwHi = 0x0205;   // 112 -> 2.5
constexpr uint16_t kBmsFwLo = 0x0404;   // 113 -> 4.4

constexpr uint16_t kModelCode = 0x1003;  // 114 -- identical on packs with different firmware

// 115-127: serial number as plain ASCII, two chars per register, big-endian, NUL padded.
// "EH153064P8S1DMA" is a common model prefix; the last 10 chars are YYMMDD + unit number.
constexpr const char* kSerial = "EH153064P8S1DMA2112222061";

// Nameplate. 204 never varies across three observed packs.
constexpr uint16_t kNameplateCapacityWh = 9600;

// 10H Prime datasheet: "Max. Charge/Discharge Power 5 kW" and
// "Peak Power (only discharging) 7 kW for 10 sec".
// NOT commissioning settings -- that reading was retracted; both are pack nameplate.
constexpr uint16_t kContinuousPowerW = 5000;  // 211, 212, 214
constexpr uint16_t kPeakDischargeW = 7000;    // 229 -- 10 s rating, not a continuous limit

int16_t clamp_i16(int32_t v) {
  if (v > 32767) return 32767;
  if (v < -32768) return -32768;
  return static_cast<int16_t>(v);
}

}  // namespace

bool LgResuPrimeModbusInverter::setup() {
  publish_identity();
  publish_alarms();

  /* Bind the Modbus server to the RS485 UART.
   *
   * Registering the FC03/FC06 workers (done by the base constructor) only makes this
   * object CAPABLE of answering -- nothing is listening on the wire until the RTU server
   * is started on Serial2. Omitting this is silent: the emulator boots, reports healthy,
   * mirrors registers, and answers nothing, because no bytes ever reach it.
   *
   * The host replay tests call FC03/FC06 directly and so cannot catch it -- they exercise
   * the protocol logic and bypass the transport entirely.
   */
  RTUutils::prepareHardwareSerial(Serial2);
  if (!rs485_begin(Name, Serial2, 9600, SERIAL_8N1)) {
    logging.println("LG RESU: rs485_begin() failed -- the emulator will not answer");
    return false;
  }
  MBserver.begin(Serial2, esp32hal->MODBUS_CORE());
  logging.println("LG RESU: Modbus RTU server listening on Serial2 @ 9600 8N1, id 15");

  /* Let the 12 V enable line drive readiness.
   *
   * 201 is derived from datalayer.system.status.inverter_allows_contactor_closing, which
   * DEFAULTS TO TRUE. Without something driving it the emulator reports the pack active
   * even with the enable line low -- measured against the real pack, which reports 1
   * (standby) in exactly that state.
   *
   * enable_sense already watches the pin; this opts it into writing the flag. It is off by
   * default because the flag also gates contactor closing in several battery drivers, so
   * only a protocol that genuinely depends on the enable line should turn it on.
   */
  enable_sense_drive_datalayer = true;

  /* Mirror mode on by default for this protocol.
   *
   * The registers this emulator must serve are a real pack's, republished by whatever is
   * mastering it (iot-rpi's lg_master.py). Deriving them from the datalayer instead means
   * serving whichever battery driver happens to be selected, and those values are not
   * self-consistent against this map -- a TestFake battery reports 206 = 23583 Wh against
   * a 9600 Wh nameplate, i.e. an SOC of 248 %.
   *
   * Turn it off (mirror_enabled() == false) only for milestones 4-5, where a Leaf pack is
   * the real thing and the datalayer IS the source of truth. Until MQTT delivers anything
   * the mirror is stale, so all power limits read 0 and the inverter is asked to move
   * nothing -- which is the right state to boot into.
   */
  /* Mirror mode is for milestones 2-3, where a real RESU is still on DC -- EXCEPT when the
   * LgResuMqtt battery driver is the source.
   *
   * That driver already feeds datalayer.battery from the real pack, so the datalayer IS
   * LG-sourced and the inverter module's normal synthesis path can run against it. Leaving the
   * mirror on in that configuration would overwrite every synthesized register with the very
   * values we are trying to check the synthesis against -- a test that cannot fail, and cannot
   * inform. Disabling it is what makes milestone 3 a real measurement rather than a re-run of
   * milestone 2 with extra steps.
   *
   * Choosing the battery type is therefore the whole switch: pick LgResuMqtt and the emulator
   * computes its registers; pick anything else and it copies them. */
  if (user_selected_battery_type == BatteryType::LgResuMqtt) {
    logging.println("LG RESU: mirror OFF -- registers synthesized from the datalayer "
                    "(LgResuMqtt battery is the source)");
  } else {
    enable_mirror(kDefaultMirrorTopic);
  }
  return true;
}

void LgResuPrimeModbusInverter::publish_identity() {
  mbPV[102] = kBatteryIdHi;
  mbPV[103] = kBatteryIdLo;
  mbPV[110] = kDcDcFwHi;
  mbPV[111] = kDcDcFwLo;
  mbPV[112] = kBmsFwHi;
  mbPV[113] = kBmsFwLo;
  mbPV[114] = kModelCode;

  // 115-127, two ASCII chars per register, big-endian, NUL padded.
  for (uint16_t i = 0; i < 13; i++) {
    const char hi = kSerial[i * 2] ? kSerial[i * 2] : '\0';
    const char lo = hi ? kSerial[i * 2 + 1] : '\0';
    mbPV[115 + i] = static_cast<uint16_t>((uint8_t)hi << 8 | (uint8_t)lo);
  }

  mbPV[141] = 1;    // unknown, static
  mbPV[142] = 256;  // unknown, static (0x0100)

  // Statics a SolarEdge reads contiguously that the Delta never asks for. Populated
  // anyway: an emulator must answer the union, and a sparse map silently returns 0.
  mbPV[101] = 0x0200;  // protocol/format version
  mbPV[136] = kNameplateCapacityWh;
  mbPV[137] = 4112;  // 411.2 V -- nominal bus voltage
  mbPV[140] = 1;
}

void LgResuPrimeModbusInverter::publish_alarms() {
  // 2001-2034. All zero in every capture to date -- nothing has ever faulted while
  // being observed, and the floor is unreachable because the inverter's Backup SoC
  // cuts above the battery's own Protection Limit. The encoding is therefore unknown;
  // zero-filling is the honest choice, not a guess at bit meanings.
  for (uint16_t r = 2001; r <= 2034; r++) {
    mbPV[r] = 0;
  }
}

void LgResuPrimeModbusInverter::mirror_config_writes() {
  /* 201 is NOT a plain mirror of 1101. Measured against the real pack 2026-08-27:
   *
   *   1101 <- 1  DOES force 201 to 1, about 0.7 s later          (proven, 4 captures)
   *   1101 <- 3  does NOT force 201 to 3                          (proven: written and
   *              echoed with the 12 V enable LOW, and 201 stayed 1)
   *
   * On a cold start the real pack reads 201 = 3 at +2.38 s -- BEFORE the inverter writes
   * anything -- so it goes active on its own once enabled. The model that fits every
   * observation is:
   *
   *   201 = 3 when the pack is ENABLED and not commanded to standby
   *   201 = 1 when the enable line is low, or standby was commanded
   *
   * So 1101 <- 1 latches standby and 1101 <- 3 only RELEASES it; readiness comes from the
   * 12 V enable line.
   *
   * Readiness is taken from datalayer.system.status.inverter_allows_contactor_closing --
   * the field that already means "the inverter has given permission" -- rather than
   * reading the GPIO here. That keeps this protocol independent of how the line is
   * sensed, and enable_sense can drive the flag by setting enable_sense_drive_datalayer.
   *
   * NOTE the flag defaults to TRUE, so with nothing driving it this reports active, which
   * is the pre-existing behaviour for inverters that do not sense an enable line.
   */
  auto it = mbPV.find(1101);
  if (it != mbPV.end()) {
    if (it->second == 1) {
      standby_commanded = true;
    } else if (it->second == 3) {
      standby_commanded = false;  // released, but readiness still comes from enable
    }
  }

  const bool enabled = datalayer.system.status.inverter_allows_contactor_closing;
  state_201 = (enabled && !standby_commanded) ? 3 : 1;
}

void LgResuPrimeModbusInverter::publish_telemetry() {
  const DATALAYER_BATTERY_TYPE& b = datalayer.battery;

  mbPV[201] = state_201;             // 1 = standby, 3 = active
  mbPV[202] = b.status.voltage_dV;   // bus volts, 0.1 V -- BE is already deciVolts, 1:1
  mbPV[203] = static_cast<uint16_t>(clamp_i16(b.status.active_power_W));  // W, signed
  mbPV[204] = kNameplateCapacityWh;
  mbPV[205] = static_cast<uint16_t>(  // SOH-adjusted full capacity, Wh
      (uint32_t)kNameplateCapacityWh * b.status.soh_pptt / 10000);
  mbPV[206] = static_cast<uint16_t>(b.status.reported_remaining_capacity_Wh);

  // 207/208 and 209/210 are 32-bit cumulative Wh, big-endian pairs.
  const uint32_t charged = (uint32_t)b.status.total_charged_battery_Wh;
  const uint32_t discharged = (uint32_t)b.status.total_discharged_battery_Wh;
  mbPV[207] = charged >> 16;
  mbPV[208] = charged & 0xFFFF;
  mbPV[209] = discharged >> 16;
  mbPV[210] = discharged & 0xFFFF;

  // Pack side. With cells_in_series set, rescale by cell count so the emulator presents
  // both sides of a converter it does not have; see the header for why this is lossless.
  if (cells_in_series > 0) {
    mbPV[215] = static_cast<uint16_t>((uint32_t)b.status.voltage_dV * kResuCellsInSeries /
                                      cells_in_series);
    mbPV[216] = static_cast<uint16_t>(
        clamp_i16((int32_t)b.status.current_dA * cells_in_series / kResuCellsInSeries));
  } else {
    mbPV[215] = b.status.voltage_dV;  // no converter: one voltage, reported on both sides
    mbPV[216] = static_cast<uint16_t>(clamp_i16(b.status.current_dA));
  }

  mbPV[217] = static_cast<uint16_t>(b.status.temperature_max_dC);  // tracks ambient
  mbPV[218] = 0xFFFF;                                              // -1, "not available" sentinel
  mbPV[219] = static_cast<uint16_t>(b.status.temperature_min_dC);  // coolest
  mbPV[227] = static_cast<uint16_t>(b.status.temperature_max_dC);  // warmest

  mbPV[220] = static_cast<uint16_t>(clamp_i16(b.status.current_dA));  // bus amps, 0.1 A signed
  mbPV[221] = b.status.reported_soc / 10;  // SOC 0.1 % (BE carries 0.01 % pptt)
  mbPV[225] = b.status.soh_pptt / 10;      // SOH 0.1 %

  mbPV[222] = 0;  // unknown flags; 222 briefly reads 4099 during an inverter reboot
  mbPV[223] = 0;
  mbPV[224] = 1;
  mbPV[228] = 1;
  mbPV[230] = (b.status.active_power_W != 0) ? 1 : 0;  // DC-DC / contactor enable
  mbPV[234] = 0;
}

void LgResuPrimeModbusInverter::publish_limits() {
  const DATALAYER_BATTERY_TYPE& b = datalayer.battery;

  // 211/212/214 are the pack's CONTINUOUS nameplate; 213 is what it will accept right
  // now. They are different fields and must not be conflated.
  mbPV[211] = kContinuousPowerW;
  mbPV[212] = kContinuousPowerW;
  mbPV[214] = kContinuousPowerW;
  mbPV[213] = static_cast<uint16_t>(
      b.status.max_charge_power_W > 65535 ? 65535 : b.status.max_charge_power_W);
  mbPV[229] = kPeakDischargeW;

  // 226 is DERIVED, not stored: the allowed charge current on the PACK side.
  //     226 == 100 * 213 / 215      (proven: mean error 0.03 A over 314 samples,
  //                                  vs 19.1 A for the bus-side rival 213/202)
  // A value inconsistent with 213 and 215 would be self-contradictory in a way a real
  // LG never is, so it is computed here rather than mapped from a datalayer field.
  const uint16_t pack_dV = mbPV[215];
  mbPV[226] = pack_dV ? static_cast<uint16_t>((uint32_t)mbPV[213] * 100 / pack_dV) : 0;
}


// ---------------------------------------------------------------------------
// Mirror mode
//
// The MQTT handler is a plain function pointer, so the active instance is held in a
// file-static. Only one inverter protocol exists at a time, which is what makes that safe.
//
// CONCURRENCY: the handler runs on the MQTT task; apply_mirror() runs on the core loop.
// The cache is therefore a flat array of uint16_t -- naturally aligned 16-bit stores are
// atomic on ESP32, so a reader can never see half a value. A reader CAN see two registers
// from slightly different moments, which for telemetry is indistinguishable from ordinary
// sampling skew and is why this needs no lock on the Modbus hot path.
// ---------------------------------------------------------------------------
namespace {

LgResuPrimeModbusInverter* g_mirror_instance = nullptr;

// Two compact windows covering everything the inverter reads: 102-142 and 201-234 in the
// low window, 2001-2034 in the high one. A flat 2 KB array beats a std::map here because
// it needs no allocation and no lock.
constexpr uint16_t kLoBase = 100, kLoCount = 150;   // 100..249
constexpr uint16_t kHiBase = 2000, kHiCount = 40;   // 2000..2039
uint16_t mirror_lo[kLoCount], mirror_hi[kHiCount];
bool mirror_lo_seen[kLoCount], mirror_hi_seen[kHiCount];
volatile uint32_t mirror_last_update_ms = 0;
/* Two guards on the liveness clock, both needed. See mirror_mqtt_handler().
 *   mirror_seen_live -- a non-retained message has arrived at some point
 *   mirror_seeded    -- a retained batch has already been allowed to seed the clock once */
volatile bool mirror_seen_live = false;
volatile bool mirror_seeded = false;
// Plain assignment, not ++: incrementing a volatile is deprecated in C++20 and the
// ESP32 build treats that as an error. We only need "has anything ever arrived".
volatile bool mirror_any_update = false;

bool mirror_store(uint16_t reg, uint16_t val) {
  if (reg >= kLoBase && reg < kLoBase + kLoCount) {
    mirror_lo[reg - kLoBase] = val;
    mirror_lo_seen[reg - kLoBase] = true;
    return true;
  }
  if (reg >= kHiBase && reg < kHiBase + kHiCount) {
    mirror_hi[reg - kHiBase] = val;
    mirror_hi_seen[reg - kHiBase] = true;
    return true;
  }
  return false;  // outside the read contract; the 1100 config block is not mirrored
}

void mirror_mqtt_handler(const char* topic, int topic_len, const char* data, int data_len,
                         bool retained) {
  // Expect <anything>/lg_<reg>/state -- pull the register out of the last "lg_" segment.
  int i = topic_len - 1;
  while (i >= 2 && !(topic[i - 2] == 'l' && topic[i - 1] == 'g' && topic[i] == '_')) i--;
  if (i < 2) return;
  const uint16_t reg = (uint16_t)atoi(topic + i + 1);
  if (!reg || data_len <= 0 || data_len > 12) return;

  char buf[13];
  memcpy(buf, data, data_len);
  buf[data_len] = '\0';
  // The publisher sends signed decimals for 203, 216-220, 226-227; Modbus carries the
  // two's-complement pattern, so a negative maps straight back onto the 16-bit word.
  const long v = strtol(buf, nullptr, 10);
  if (v < -32768 || v > 65535) return;

  if (!mirror_store(reg, (uint16_t)(v < 0 ? v + 65536 : v))) return;

  /* Storing is unconditional; STAMPING THE LIVENESS CLOCK is not.
   *
   * The Pi publishes every register retained, which is deliberate: an unpopulated mbPV
   * answers reads with 0 (std::map::operator[] default-inserts), so a cold start with no
   * seed would present the Delta a battery at 0 V. Retained data prevents that.
   *
   * But the broker also replays every retained topic to a LATE SUBSCRIBER, and BE
   * re-subscribes on every MQTT_EVENT_CONNECTED. Treating that replay as a fresh update
   * meant a flapping link with a DEAD publisher looked permanently alive -- the one case
   * the staleness check exists for. So retained data seeds the clock exactly ONCE, at the
   * first batch, and every later replay is storage only.
   *
   * Live (non-retained) messages always stamp: they are proof the master just read the pack.
   */
  if (retained) {
    // Seed the clock only if nothing live has ever arrived, and only for the first batch.
    // Both guards are load-bearing: without the first, a replay after a healthy period
    // defers staleness; without the second, repeated reconnects after a cold start into a
    // dead master defer it forever.
    if (mirror_seen_live || mirror_seeded) return;
    mirror_seeded = true;
  } else {
    mirror_seen_live = true;
  }
  mirror_last_update_ms = millis();
  mirror_any_update = true;
}

}  // namespace

bool LgResuPrimeModbusInverter::enable_mirror(const char* topic_filter) {
  if (!topic_filter) return false;
  memset(mirror_lo_seen, 0, sizeof(mirror_lo_seen));
  memset(mirror_hi_seen, 0, sizeof(mirror_hi_seen));
  mirror_any_update = false;
  mirror_seen_live = false;
  mirror_seeded = false;
  g_mirror_instance = this;
  if (!mqtt_register_subscription(topic_filter, mirror_mqtt_handler)) {
    logging.println("LG mirror: could not register the MQTT subscription");
    return false;
  }
  mirror_on = true;
  logging.printf("LG mirror: serving registers from %s\n", topic_filter);
  return true;
}

bool LgResuPrimeModbusInverter::mirror_is_fresh() const {
  if (!mirror_any_update) return false;
  return (millis() - mirror_last_update_ms) < kMirrorStaleMs;
}

void LgResuPrimeModbusInverter::apply_mirror() {
  for (uint16_t i = 0; i < kLoCount; i++)
    if (mirror_lo_seen[i]) mbPV[kLoBase + i] = mirror_lo[i];
  for (uint16_t i = 0; i < kHiCount; i++)
    if (mirror_hi_seen[i]) mbPV[kHiBase + i] = mirror_hi[i];

  /* 201 is the exception to mirroring.
   *
   * It is the read side of 1101, and the INVERTER writes 1101 to us. If we served the real
   * pack's 201 instead, the Delta would command standby and see the battery still report
   * active -- an inconsistency a real pack never shows. So its own write wins here.
   *
   * ARCHITECTURAL GAP this exposes: in mirror mode the Delta's config writes reach the
   * emulator but NOT the real battery behind it. The real pack therefore never receives
   * its graceful 1101<-1 shutdown command. Relaying those writes through to the master is
   * an open piece of milestone 2; until it exists, iot-rpi must manage the real pack's
   * state itself.
   */
  mbPV[201] = state_201;

  if (!mirror_is_fresh()) {
    /* Stale mirrored data. Report the pack as unable to move power, rather than going
     * silent.
     *
     * Silence WOULD be safe -- a Delta talking to nothing polls patiently for at least
     * 235 s with no fault, no lockout and no retry escalation (measured 2026-08-26 with
     * the battery physically off the bus). But silence also makes the battery vanish from
     * the inverter's UI and tells it nothing.
     *
     * Zero limits keep the pack visibly present and healthy while actively asking the
     * inverter to stop. But limits are only a REQUEST, and it is unproven that the Delta
     * obeys them at all -- the droop law says it never commands power in the first place.
     * The SOC it reads is the one lever it is proven to act on, so that is forced too: a
     * frozen SOC with zero limits would leave the discharge floor keyed off a stuck value,
     * defeating the protection entirely.
     */
    mbPV[213] = 0;  // instantaneous allowed charge
    mbPV[226] = 0;  // its pack-side current form
    mbPV[229] = 0;  // peak discharge
    mbPV[211] = 0;
    mbPV[212] = 0;
    mbPV[214] = 0;

    // SOC -> 10.1 %, on BOTH registers it can be read from: 221 is the pack's own figure,
    // and SOC = 206 / 205 is what a SolarEdge derives. Letting them disagree would be a
    // state no real pack shows.
    mbPV[221] = kStaleSocTenthPct;
    auto full = mbPV.find(205);
    if (full == mbPV.end() || full->second == 0) full = mbPV.find(204);
    const uint16_t full_Wh =
        (full != mbPV.end() && full->second) ? full->second : kNameplateCapacityWh;
    mbPV[206] = static_cast<uint16_t>((uint32_t)full_Wh * kStaleSocTenthPct / 1000);

    static uint32_t last_warn = 0;
    if (millis() - last_warn > 10000) {
      last_warn = millis();
      logging.printf("LG mirror: STALE (%lu ms since last update) -- limits 0, SOC 10.1%%\n",
                     (unsigned long)(millis() - mirror_last_update_ms));
    }
  }
}

void LgResuPrimeModbusInverter::update_values() {
  mirror_config_writes();

  if (mirror_on) {
    // Serve the real battery's registers verbatim. The datalayer mapping is skipped
    // entirely: it cannot reproduce registers that have no datalayer equivalent, and
    // re-deriving values we already have would only add error.
    apply_mirror();
    return;
  }

  publish_telemetry();
  publish_limits();

  /* NOTE -- pack side vs bus side.
   *
   * A real RESU Prime reports BOTH sides of its internal DC-DC: 202/220 are the
   * inverter-facing bus (~410 V) and 215/216 the pack behind the converter (~170 V),
   * and each side's V*I reproduces 203 to about 1 %.
   *
   * A Battery-Emulator install has no such converter, so there is only one voltage to
   * report and 202 == 215 here. That is self-consistent and the Delta accepts the read
   * contract, but it is NOT what a real LG looks like: on a real pack 226 (= 100*213/215)
   * lands at 29-34 A, whereas with 202 == 215 it will land near the bus-side figure
   * instead. If the Delta ever cross-checks the two sides, this is where it would notice.
   * Flagged rather than papered over.
   */
}
