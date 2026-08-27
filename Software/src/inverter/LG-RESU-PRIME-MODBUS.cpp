#include "LG-RESU-PRIME-MODBUS.h"

#include "../datalayer/datalayer.h"
#include "../devboard/utils/logging.h"

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
  // 1101 is the write side of read-register 201. Proven passively: the Delta writes
  // 1101<-1 about 3 s before it stops polling and 201 follows to 1 ~0.7 s later.
  // On a cold battery it also writes 1101<-1 then 1101<-3 at +7.18/+7.68 s after boot
  // ("apply config and go active"), which this mirror reproduces for free.
  auto it = mbPV.find(1101);
  if (it != mbPV.end() && (it->second == 1 || it->second == 3)) {
    state_201 = it->second;
  }
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

void LgResuPrimeModbusInverter::update_values() {
  mirror_config_writes();
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
