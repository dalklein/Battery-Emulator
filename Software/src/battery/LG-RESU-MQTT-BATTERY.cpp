#include "LG-RESU-MQTT-BATTERY.h"

#include <stdlib.h>
#include <string.h>

#include "../devboard/mqtt/mqtt.h"
#include "../devboard/utils/events.h"
#include "../devboard/utils/logging.h"

namespace {

/* The handler is a plain function pointer, so the active instance lives in a file-static.
 * Only one battery of this type exists at a time, which is what makes that safe -- the same
 * argument the inverter module's mirror relies on. */
LgResuMqttBattery* g_instance = nullptr;

/* Registers lg_master.py publishes as SIGNED. Everything else is unsigned.
 *
 * NB this list is the MASTER's, not the sniffer's: the sniffer's --mqtt-signed covers only the
 * 31 registers the Pi publishes, and 203 belongs to the ESP32's half. Copying the sniffer's
 * list here published 203 as 64534 instead of -1002 W once already. */
bool is_signed(uint16_t reg) {
  switch (reg) {
    case 203:
    case 216:
    case 217:
    case 218:
    case 219:
    case 220:
    case 226:
    case 227:
      return true;
    default:
      return false;
  }
}

/* The topic is NOT null-terminated -- esp-mqtt hands over pointer + length. Pull the register
 * out of the last "lg_" segment, the same way the inverter module's mirror handler does. */
void on_message(const char* topic, int topic_len, const char* data, int data_len,
                bool retained) {
  // A retained replay is as good as a live message here: this driver's own staleness check
  // (source_is_stale) is what decides whether the data may be used, and it does not care
  // how the value arrived.
  (void)retained;
  if (!g_instance || !topic || !data) return;
  int i = topic_len - 1;
  while (i >= 2 && !(topic[i - 2] == 'l' && topic[i - 1] == 'g' && topic[i] == '_')) i--;
  if (i < 2) return;
  const uint16_t reg = (uint16_t)atoi(topic + i + 1);
  if (!reg || data_len <= 0 || data_len > 12) return;

  char buf[13];
  memcpy(buf, data, data_len);
  buf[data_len] = '\0';
  long raw = strtol(buf, nullptr, 10);
  /* lg_master publishes signed registers as negative decimals, so strtol is usually enough.
   * The is_signed() table covers a publisher that sends the raw 16-bit word instead. */
  if (is_signed(reg) && raw > 32767) raw -= 65536;
  if (raw < -32768 || raw > 65535) return;
  g_instance->ingest(reg, (int32_t)raw, millis());
}

}  // namespace

void LgResuMqttBattery::setup(void) {
  datalayer.battery.info.max_design_voltage_dV = 4500;  // 450.0 V, RESU charge maximum (reg 1105)
  datalayer.battery.info.min_design_voltage_dV = 3500;  // 350.0 V, RESU discharge minimum (reg 1107)
  datalayer.battery.info.total_capacity_Wh = 9600;
  datalayer.battery.info.reported_total_capacity_Wh = 9600;

  /* Name the protocol for the dashboard.
   *
   * datalayer.system.info.battery_protocol is what the web UI prints as "Battery protocol:",
   * and every driver has to fill it in itself -- there is no fallback to Name. Omitting it
   * left the dashboard showing a bare "Battery protocol:" with nothing after it, which reads
   * as a failed setup rather than a driver that simply never set the string.
   */
  strncpy(datalayer.system.info.battery_protocol, Name, 63);
  datalayer.system.info.battery_protocol[63] = '\0';

  g_instance = this;
  if (!mqtt_register_subscription(kDefaultTopic, on_message)) {
    // Same reasoning as the staleness branch: no CAN is involved, and a failed subscription
    // means the values will never be anything but stale.
    set_event(EVENT_STALE_VALUE, 0);
    logging.println("LG RESU MQTT: could not subscribe -- no state will arrive");
    return;
  }
  logging.printf("LG RESU MQTT: sourcing battery state from %s\n", kDefaultTopic);
}

void LgResuMqttBattery::ingest(uint16_t r, int32_t value, uint32_t now_ms) {
  if (r < kFirst || r > kLast) return;
  reg[r - kFirst] = value;
  seen[r - kFirst] = true;
  last_update_ms = now_ms;
  any_seen = true;
}

bool LgResuMqttBattery::source_is_stale(uint32_t now_ms) const {
  if (!any_seen) return true;
  return (uint32_t)(now_ms - last_update_ms) > kStaleMs;
}

void LgResuMqttBattery::apply_to(DATALAYER_BATTERY_TYPE& b, uint32_t now_ms) {
  /* Dead source: withdraw everything that could move power.
   *
   * Zeroing the limits is the whole point. Leaving the last-known values in place would let
   * the inverter keep commanding power against a battery nobody is reading any more -- which
   * is exactly the failure the 2026-08-27 pack-sleep incident showed is reachable: the source
   * can vanish and not come back without a human. */
  if (source_is_stale(now_ms)) {
    b.status.max_charge_power_W = 0;
    b.status.max_discharge_power_W = 0;
    if (allows_contactor_closing) *allows_contactor_closing = false;
    /* EVENT_STALE_VALUE, not EVENT_CAN_BATTERY_MISSING.
     *
     * There is no CAN bus anywhere in this driver, so the CAN event was simply the wrong
     * name for the condition -- it put "Battery not sending messages via CAN for the last
     * 60 seconds. Check wiring!" on the status page of a system with no battery CAN wiring
     * to check. EVENT_STALE_VALUE is also ERROR level, so the safe consequence is unchanged:
     * system_status goes to FAULT and safety.cpp zeroes the power limits.
     */
    set_event(EVENT_STALE_VALUE, 0);
    return;
  }

  if (have(202)) b.status.voltage_dV = (uint16_t)get(202);   // DC bus, 0.1 V -- see header
  if (have(220)) b.status.current_dA = (int16_t)get(220);    // bus amps, 0.1 A signed
  if (have(203)) b.status.active_power_W = get(203);         // W, signed: + charge, - discharge

  if (have(206)) b.status.reported_remaining_capacity_Wh = (uint32_t)get(206);
  if (have(206)) b.status.remaining_capacity_Wh = (uint32_t)get(206);
  if (have(204)) {
    b.info.total_capacity_Wh = (uint32_t)get(204);
    b.info.reported_total_capacity_Wh = (uint32_t)get(204);
  }

  /* 221 is SOC in 0.1 %; the datalayer carries 0.01 % (pptt).
   *
   * NOT 206. Register 206 is REMAINING ENERGY IN Wh, and it is easy to mistake for SOC because
   * a 9600 Wh pack puts Wh and 0.01 %-of-full in the same numeric range. Proven 2026-08-27:
   * with 204 = 9600 and 221 = 396 (39.6 %), 206 read 3806, and 9600 x 0.396 = 3802 Wh. Reading
   * 206 as SOC would have given 38.06 %, contradicting 221 by 1.5 points. */
  if (have(221)) {
    b.status.reported_soc = (uint16_t)(get(221) * 10);
    b.status.real_soc = (uint16_t)(get(221) * 10);
  }
  if (have(225)) b.status.soh_pptt = (uint16_t)(get(225) * 10);

  /* The temperature set is 217 (max) and 219 (min) -- TWO registers, not three.
   *
   * temperature_max_dC was fed from 227 until 2026-09-01. 227 is NOT a temperature: it is
   * the pack-side max DISCHARGE current in 0.1 A, retracted 2026-08-28 after 227 x 215 came
   * out at 5159 W and 5156 W across a 10 % voltage swing, and after 227 = 335 was seen to
   * exceed 217 = 275 -- a "warmest cell" hotter than the reported pack maximum.
   *
   * This mattered precisely because it looked right: 227 = 335 reads as a plausible 33.5 C,
   * so nothing about the served value would have looked wrong in a capture. The inverter
   * module derives 227 itself in publish_limits(), so feeding it in here also double-counted.
   */
  if (have(217)) b.status.temperature_max_dC = (int16_t)get(217);
  if (have(219)) b.status.temperature_min_dC = (int16_t)get(219);

  /* 213 is what the pack will accept RIGHT NOW; 214 is the continuous nameplate. Different
   * fields -- conflating them was an early error worth not repeating. */
  if (have(213)) b.status.max_charge_power_W = (uint32_t)get(213);
  if (have(214)) b.status.max_discharge_power_W = (uint32_t)get(214);

  // 207/208 and 209/210 are 32-bit cumulative Wh, big-endian pairs.
  if (have(207) && have(208))
    b.status.total_charged_battery_Wh = (int32_t)(((uint32_t)get(207) << 16) | (uint32_t)get(208));
  if (have(209) && have(210))
    b.status.total_discharged_battery_Wh =
        (int32_t)(((uint32_t)get(209) << 16) | (uint32_t)get(210));

  /* Clear the stale event on the way back up.
   *
   * The driver only ever RAISED it before. That was survivable while safety.cpp was
   * re-raising a CAN event every cycle anyway, but with that fixed a single startup gap --
   * and there is always one, since setup() runs about 5 s before MQTT connects -- would
   * otherwise leave the system latched in FAULT with both limits at zero for good.
   */
  clear_event(EVENT_STALE_VALUE);
  if (allows_contactor_closing) *allows_contactor_closing = true;
}

void LgResuMqttBattery::update_values() {
  apply_to(*datalayer_battery, millis());
}
