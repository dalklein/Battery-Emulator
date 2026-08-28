#ifndef LG_RESU_MQTT_BATTERY_H
#define LG_RESU_MQTT_BATTERY_H

/* LG RESU Prime, sourced over MQTT -- the milestone-3 battery driver.
 *
 * WHY THIS EXISTS
 *
 * Milestone 2 proved the Delta accepts Battery-Emulator, but it did so with the inverter
 * module in MIRROR mode: every register served to the inverter was COPIED verbatim from the
 * real pack. That validates the protocol and validates nothing about whether the emulator can
 * COMPUTE those values, because it never had to.
 *
 * This driver closes that gap. It feeds datalayer.battery from the real pack's registers
 * (republished to MQTT by lg_master.py on the battery's own RS485 segment), so the inverter
 * module can run its normal datalayer synthesis path -- the same code milestone 4 depends on --
 * while the real pack is still present as ground truth for every derived value.
 *
 * Milestone 4 exercises that arithmetic for the first time with NO reference to check against.
 * This is the step that makes it checkable.
 *
 * ⚠️ WHAT IT CANNOT VALIDATE
 *
 * The synthesis models a NATIVE pack, where one voltage is rescaled by series cell count:
 *
 *     215 = 202 * kResuCellsInSeries / cells_in_series
 *
 * A real RESU has a DC-DC, so its bus voltage (~410 V, set by the converter's rails) and its
 * pack voltage (~154 V) are INDEPENDENT -- no cell-count ratio relates them. Register 202 must
 * remain the bus voltage, since the Delta measures that rail itself. It follows that 215, 216
 * and the derived 226 CANNOT match the real pack while this driver is the source.
 *
 * That is a property of the model, not a defect, and it is the honest limit of this milestone:
 * everything else (202, 203, 206, 207-210, 213, 214, 217, 219, 220, 221, 225, 227) is checkable
 * against the real pack; those three are not.
 *
 * ⚠️ NOT FOR PRODUCTION USE. A battery whose state arrives over WiFi has no business closing
 * contactors on its own. Staleness is treated as a hard fault -- see kStaleMs.
 */

#include "../datalayer/datalayer.h"
#include "Battery.h"

class LgResuMqttBattery : public Battery {
 public:
  LgResuMqttBattery() {
    datalayer_battery = &datalayer.battery;
    allows_contactor_closing = &datalayer.system.status.battery_allows_contactor_closing;
  }

  static constexpr const char* Name = "LG RESU Prime (state sourced over MQTT)";

  virtual void setup(void);
  virtual void update_values();
  virtual const char* interface_name() { return "MQTT"; }

  /** Default topic pattern published by lg_master.py. '+' matches the lg_NNN level. */
  static constexpr const char* kDefaultTopic = "lg/master/sensor/+/state";

  /** No update for this long and the source is considered dead: limits go to zero and the
   *  battery withdraws permission to close contactors. The republish interval is 5 s, so this
   *  tolerates two missed cycles. Matches the inverter module's mirror-liveness window. */
  static constexpr uint32_t kStaleMs = 15000;

  /** Feed one register in, as the MQTT handler does. Exposed so the mapping can be tested on
   *  the host without an MQTT broker. */
  void ingest(uint16_t reg, int32_t value, uint32_t now_ms);

  /** Map whatever has been ingested onto a datalayer. Split out from update_values() so tests
   *  can drive it against a scratch struct rather than the global. */
  void apply_to(DATALAYER_BATTERY_TYPE& b, uint32_t now_ms);

  bool source_is_stale(uint32_t now_ms) const;

 private:
  DATALAYER_BATTERY_TYPE* datalayer_battery;
  bool* allows_contactor_closing;

  // Only the registers the datalayer actually consumes are stored.
  static const uint16_t kFirst = 202;
  static const uint16_t kLast = 227;
  static const int kCount = kLast - kFirst + 1;

  int32_t reg[kCount] = {0};
  bool seen[kCount] = {false};
  uint32_t last_update_ms = 0;
  bool any_seen = false;

  bool have(uint16_t r) const { return r >= kFirst && r <= kLast && seen[r - kFirst]; }
  int32_t get(uint16_t r) const { return reg[r - kFirst]; }
};

#endif
