#include <gtest/gtest.h>

#include "../../Software/src/battery/LG-RESU-MQTT-BATTERY.h"
#include "../../Software/src/datalayer/datalayer.h"

namespace {

// A pack mid-discharge, taken from the real LG on 2026-08-27.
void feed_healthy(LgResuMqttBattery& b, uint32_t t) {
  b.ingest(202, 4101, t);   // 410.1 V bus
  b.ingest(203, -3836, t);  // discharging 3836 W
  b.ingest(204, 9600, t);   // nameplate Wh
  b.ingest(206, 3806, t);   // remaining Wh  <-- NOT SOC
  b.ingest(213, 5000, t);
  b.ingest(214, 5000, t);
  b.ingest(219, 279, t);    // 27.9 C min
  b.ingest(220, -252, t);   // -25.2 A bus
  b.ingest(221, 396, t);    // 39.6 % SOC
  b.ingest(225, 1000, t);   // 100.0 % SOH
  b.ingest(227, 292, t);    // 29.2 C max
}

TEST(LgResuMqtt, MapsRegistersOntoDatalayer) {
  LgResuMqttBattery b;
  DATALAYER_BATTERY_TYPE d{};
  feed_healthy(b, 1000);
  b.apply_to(d, 1000);

  EXPECT_EQ(d.status.voltage_dV, 4101);
  EXPECT_EQ(d.status.current_dA, -252);
  EXPECT_EQ(d.status.active_power_W, -3836);
  EXPECT_EQ(d.status.max_charge_power_W, 5000u);
  EXPECT_EQ(d.status.max_discharge_power_W, 5000u);
  EXPECT_EQ(d.status.temperature_min_dC, 279);
  EXPECT_EQ(d.status.temperature_max_dC, 292);
  EXPECT_EQ(d.status.soh_pptt, 10000);
}

// *** Regression test for a mistake made repeatedly on 2026-08-27. ***
// 206 is REMAINING ENERGY IN Wh; 221 is the SOC. They are easy to confuse because a 9600 Wh
// pack puts Wh and 0.01%-of-full in the same numeric range. The arithmetic separates them:
// 9600 Wh x 39.6% = 3802 Wh, which matches 206 = 3806. Reading 206 as SOC gives 38.06%,
// contradicting 221's 39.6%.
TEST(LgResuMqtt, Reg206IsRemainingWhNotSoc) {
  LgResuMqttBattery b;
  DATALAYER_BATTERY_TYPE d{};
  feed_healthy(b, 1000);
  b.apply_to(d, 1000);

  EXPECT_EQ(d.status.reported_remaining_capacity_Wh, 3806u) << "206 must map to remaining Wh";
  EXPECT_EQ(d.status.reported_soc, 3960) << "SOC must come from 221 (0.1%) scaled to pptt";
  EXPECT_NE(d.status.reported_soc, 3806) << "SOC must NOT be taken from 206";

  // Internal consistency: capacity x SOC reproduces the remaining energy, within rounding.
  const uint32_t implied = d.info.total_capacity_Wh * d.status.reported_soc / 10000;
  EXPECT_NEAR((double)implied, (double)d.status.reported_remaining_capacity_Wh, 10.0);
}

TEST(LgResuMqtt, SignedPowerAndCurrentSurviveAsNegatives) {
  LgResuMqttBattery b;
  DATALAYER_BATTERY_TYPE d{};
  feed_healthy(b, 1000);
  b.apply_to(d, 1000);
  EXPECT_LT(d.status.active_power_W, 0) << "discharge must stay negative, not wrap to 61700";
  EXPECT_LT(d.status.current_dA, 0);
}

TEST(LgResuMqtt, ThirtyTwoBitEnergyPairsAreBigEndian) {
  LgResuMqttBattery b;
  DATALAYER_BATTERY_TYPE d{};
  feed_healthy(b, 1000);
  b.ingest(207, 0x0001, 1000);
  b.ingest(208, 0x86A0, 1000);  // 0x000186A0 = 100000 Wh
  b.ingest(209, 0x0000, 1000);
  b.ingest(210, 0xC350, 1000);  // 50000 Wh
  b.apply_to(d, 1000);
  EXPECT_EQ(d.status.total_charged_battery_Wh, 100000);
  EXPECT_EQ(d.status.total_discharged_battery_Wh, 50000);
}

// A battery whose state arrives over WiFi must fail closed. If the source dies, leaving the
// last-known limits in place would let the inverter keep commanding power against a pack
// nobody is reading -- and the 2026-08-27 pack-sleep incident proved the source CAN vanish
// and not return without a human.
TEST(LgResuMqtt, StaleSourceZeroesLimits) {
  LgResuMqttBattery b;
  DATALAYER_BATTERY_TYPE d{};
  feed_healthy(b, 1000);
  b.apply_to(d, 1000);
  ASSERT_EQ(d.status.max_charge_power_W, 5000u);

  b.apply_to(d, 1000 + LgResuMqttBattery::kStaleMs + 1);
  EXPECT_EQ(d.status.max_charge_power_W, 0u);
  EXPECT_EQ(d.status.max_discharge_power_W, 0u);
  EXPECT_FALSE(datalayer.system.status.battery_allows_contactor_closing);
}

TEST(LgResuMqtt, StaleBeforeAnyDataArrives) {
  LgResuMqttBattery b;
  EXPECT_TRUE(b.source_is_stale(0));
  EXPECT_TRUE(b.source_is_stale(100000));
}

TEST(LgResuMqtt, FreshDataAfterStalenessRecovers) {
  LgResuMqttBattery b;
  DATALAYER_BATTERY_TYPE d{};
  feed_healthy(b, 1000);
  b.apply_to(d, 1000 + LgResuMqttBattery::kStaleMs + 1);
  ASSERT_EQ(d.status.max_charge_power_W, 0u);

  feed_healthy(b, 100000);
  b.apply_to(d, 100000);
  EXPECT_EQ(d.status.max_charge_power_W, 5000u);
  EXPECT_TRUE(datalayer.system.status.battery_allows_contactor_closing);
}

}  // namespace
