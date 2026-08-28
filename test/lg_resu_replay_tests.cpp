#include <gtest/gtest.h>

#include <cstdio>
#include <fstream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#include "../Software/src/datalayer/datalayer.h"
#include "../Software/src/inverter/LG-RESU-PRIME-MODBUS.h"

/* Replays real inverter<->battery traffic through the emulator.
 *
 * lg_resu/lg_replay_corpus.jsonl is 184 answered exchanges captured 2026-08-26 between
 * a Delta E6-TL-US and a real LG RESU10H Prime, spanning one complete cold start: all
 * 22 fn3 read patterns and all 8 fn6 writes of the boot handshake, with the battery's
 * actual replies.
 *
 * WHAT IS AND IS NOT LOAD-BEARING HERE
 *
 * Registers the emulator passes straight through from the datalayer cannot be
 * meaningfully checked by seeding the datalayer from the same capture -- that would
 * assert 1 == 1. The assertions that carry weight are:
 *
 *   * coverage      -- every pattern the inverter asks for is answered, with the right
 *                      length and no exception. A sparse map silently returns 0 instead
 *                      of erroring, so absence of a crash proves nothing on its own.
 *   * fn6 echo      -- byte-for-byte, the thing that breaks the boot handshake
 *   * static block  -- identity, firmware, ASCII serial, nameplate, alarm block: these
 *                      are ours to get right and are compared byte-for-byte
 *   * derived regs  -- 226 is computed, not stored. Fed the real 213 and 215, it must
 *                      land on the real 226.
 */

namespace {

class ReplayInverter : public LgResuPrimeModbusInverter {
 public:
  using LgResuPrimeModbusInverter::FC03;
  using LgResuPrimeModbusInverter::FC06;
  using LgResuPrimeModbusInverter::mbPV;
};

std::vector<uint8_t> unhex(const std::string& h) {
  std::vector<uint8_t> v;
  for (size_t i = 0; i + 1 < h.size(); i += 2)
    v.push_back((uint8_t)std::stoul(h.substr(i, 2), nullptr, 16));
  return v;
}

std::string hex(const std::vector<uint8_t>& v) {
  std::ostringstream o;
  for (uint8_t b : v) {
    char buf[3];
    snprintf(buf, sizeof(buf), "%02x", b);
    o << buf;
  }
  return o.str();
}

std::vector<uint8_t> bytes_of(ModbusMessage& m) {
  return std::vector<uint8_t>(m.data(), m.data() + m.size());
}

struct Exchange {
  std::vector<uint8_t> req, rsp;
  uint8_t fn;
  uint16_t start, count;
};

std::string field(const std::string& line, const char* key) {
  const std::string k = std::string("\"") + key + "\": \"";
  size_t a = line.find(k);
  if (a == std::string::npos) return "";
  a += k.size();
  return line.substr(a, line.find('"', a) - a);
}

std::vector<Exchange> load_corpus() {
  std::vector<Exchange> out;
  std::ifstream f(std::string(TEST_LG_RESU_DIR) + "/lg_replay_corpus.jsonl");
  std::string line;
  while (std::getline(f, line)) {
    if (line.empty()) continue;
    Exchange e;
    e.req = unhex(field(line, "req"));
    e.rsp = unhex(field(line, "rsp"));
    if (e.req.size() < 8) continue;
    e.fn = e.req[1];
    e.start = (uint16_t)(e.req[2] << 8 | e.req[3]);
    e.count = (uint16_t)(e.req[4] << 8 | e.req[5]);
    out.push_back(e);
  }
  return out;
}

// Strip the two trailing CRC bytes: FC03/FC06 build the payload, the RTU layer adds CRC.
std::vector<uint8_t> payload(const std::vector<uint8_t>& frame) {
  return std::vector<uint8_t>(frame.begin(), frame.end() - 2);
}

ModbusMessage make_request(const Exchange& e) {
  return ModbusMessage(e.req[0], e.fn, e.start, e.count);
}

}  // namespace

TEST(LgResuReplay, CorpusLoads) {
  auto c = load_corpus();
  ASSERT_GE(c.size(), 180u) << "corpus missing or truncated at " << TEST_LG_RESU_DIR;
  int reads = 0, writes = 0;
  for (auto& e : c) (e.fn == 0x03 ? reads : writes)++;
  EXPECT_GT(reads, 150);
  EXPECT_EQ(writes, 8) << "a cold-start handshake has exactly 8 fn6 writes";
}

TEST(LgResuReplay, AnswersEveryCapturedRequestWithTheRightShape) {
  ReplayInverter inv;
  inv.setup();
  inv.update_values();

  std::map<uint16_t, int> unanswered;
  for (auto& e : load_corpus()) {
    if (e.fn != 0x03) continue;
    ModbusMessage req = make_request(e);
    ModbusMessage rsp = inv.FC03(req);
    auto got = bytes_of(rsp);

    ASSERT_GE(got.size(), 3u) << "empty response to fn3 start=" << e.start;
    if (got[1] & 0x80) {
      unanswered[e.start]++;
      continue;
    }
    EXPECT_EQ(got[0], e.req[0]) << "wrong server id answering start=" << e.start;
    EXPECT_EQ(got[1], 0x03);
    EXPECT_EQ(got[2], e.count * 2) << "byte count wrong for start=" << e.start;
    EXPECT_EQ(got.size(), payload(e.rsp).size())
        << "response length differs from the real battery for start=" << e.start;
  }
  EXPECT_TRUE(unanswered.empty()) << "the inverter asks for registers we refuse to serve";
}

TEST(LgResuReplay, EchoesEveryFn6WriteOfTheHandshake) {
  ReplayInverter inv;
  inv.setup();

  int writes = 0;
  for (auto& e : load_corpus()) {
    if (e.fn != 0x06) continue;
    writes++;
    ModbusMessage req = make_request(e);
    ModbusMessage rsp = inv.FC06(req);
    EXPECT_EQ(hex(bytes_of(rsp)), hex(payload(e.rsp)))
        << "fn6 reg=" << e.start << " val=" << e.count << " not echoed as the real battery does";
  }
  EXPECT_EQ(writes, 8);
}

TEST(LgResuReplay, StaticBlockMatchesTheRealBatteryByteForByte) {
  ReplayInverter inv;
  inv.setup();
  inv.update_values();

  // Registers whose values are ours to get right and never change: identity, firmware,
  // model code, ASCII serial, the two static flags, and the alarm block.
  const uint16_t kStatic[] = {102, 110, 112, 114, 115, 141, 142, 2001, 2004, 2011, 2021, 2031};

  int compared = 0;
  for (auto& e : load_corpus()) {
    if (e.fn != 0x03) continue;
    bool is_static = false;
    for (uint16_t s : kStatic)
      if (e.start == s) is_static = true;
    if (!is_static) continue;

    ModbusMessage req = make_request(e);
    ModbusMessage rsp = inv.FC03(req);
    EXPECT_EQ(hex(bytes_of(rsp)), hex(payload(e.rsp)))
        << "static read start=" << e.start << " count=" << e.count
        << " differs from the real battery";
    compared++;
  }
  EXPECT_GT(compared, 50) << "expected many static reads in the corpus";
}

TEST(LgResuReplay, DerivedRegister226MatchesTheRealBattery) {
  // 226 is the max allowed charge CURRENT on the pack side. The emulator COMPUTES it as
  // 100 * 213 / 215 rather than storing it, because a value inconsistent with 213 and 215
  // would be self-contradictory in a way a real LG never is.
  //
  // This must drive the module's own code path -- an earlier version of this test did the
  // arithmetic inline and therefore passed even when publish_limits() was broken.
  //
  // KNOWN LIMIT OF THIS TEST, established by mutation testing:
  //   caught      : wrong constant (100 -> 50), zeroed output, missing computation
  //   NOT caught  : deriving 226 from 202 (bus side) instead of 215 (pack side)
  // On a real RESU those differ by ~19 A, and the pack-side derivation is proven. But the
  // emulator has no DC-DC, so mbPV[202] == mbPV[215] and the substitution is a no-op here.
  // Nothing in a host test can distinguish them; only a real two-sided pack would.
  ReplayInverter inv;
  inv.setup();
  inv.disable_mirror();  // this exercises the datalayer path, not the mirror

  // Recover real (213, 215, 226) triples from the capture. 213x9 covers 213..221 and
  // 222x8 covers 222..229, so both reads are needed for one triple.
  auto corpus = load_corpus();
  std::vector<std::pair<uint16_t, uint16_t>> limits;  // (213, 215)
  std::vector<uint16_t> real226;
  for (auto& e : corpus) {
    if (e.fn != 0x03) continue;
    if (e.start == 213 && e.count == 9) {
      auto q = payload(e.rsp);
      limits.emplace_back((uint16_t)(q[3] << 8 | q[4]), (uint16_t)(q[3 + 4] << 8 | q[3 + 5]));
    } else if (e.start == 222 && e.count == 8) {
      auto q = payload(e.rsp);
      real226.push_back((uint16_t)(q[3 + 8] << 8 | q[3 + 9]));
    }
  }
  ASSERT_FALSE(limits.empty());
  ASSERT_FALSE(real226.empty());

  const size_t n = std::min(limits.size(), real226.size());
  int checked = 0;
  for (size_t i = 0; i < n; i++) {
    const uint16_t r213 = limits[i].first, r215 = limits[i].second, r226 = real226[i];
    if (!r215 || !r226) continue;

    // Drive the module: these are the datalayer fields 213 and 215 are mapped from.
    datalayer.battery.status.max_charge_power_W = r213;
    datalayer.battery.status.voltage_dV = r215;
    inv.update_values();

    ASSERT_EQ(inv.mbPV[213], r213) << "precondition: 213 did not take the captured value";
    ASSERT_EQ(inv.mbPV[215], r215) << "precondition: 215 did not take the captured value";
    EXPECT_NEAR(inv.mbPV[226], r226, 2)
        << "emulator 226 = " << inv.mbPV[226] << " but the real battery reported " << r226
        << " for 213=" << r213 << " 215=" << r215;
    checked++;
  }
  EXPECT_GT(checked, 5) << "expected several usable (213,215,226) triples in the corpus";
}

// --- mirror mode -----------------------------------------------------------
//
// For milestones 2 and 3 a REAL RESU is still on DC; iot-rpi masters it and republishes
// every register to MQTT, and the emulator serves those values verbatim. These tests use
// the host MQTT stub to inject messages exactly as the broker would.

extern void mqtt_test_reset_subscriptions();
extern int mqtt_test_deliver(const char* topic, const char* payload, bool retained = false);
extern size_t mqtt_test_subscription_count();
extern uint64_t current_time;  // emul/time.cpp -- the host's millis() source

namespace {
void feed_from_corpus(int max_regs = 1000, bool retained = false) {
  // Replay the real battery's register values through MQTT, as the Pi would publish them.
  std::map<uint16_t, uint16_t> regs;
  for (auto& e : load_corpus()) {
    if (e.fn != 0x03) continue;
    auto p = payload(e.rsp);
    if ((int)p.size() < 3 + e.count * 2) continue;
    for (uint16_t i = 0; i < e.count; i++)
      regs[e.start + i] = (uint16_t)(p[3 + i * 2] << 8 | p[4 + i * 2]);
  }
  int n = 0;
  for (auto& kv : regs) {
    if (n++ >= max_regs) break;
    char topic[64], val[16];
    snprintf(topic, sizeof(topic), "lg/master/sensor/lg_%u/state", kv.first);
    // publish signed for the registers the master publishes signed
    const bool sgn = (kv.first == 203 || kv.first == 216 || kv.first == 218 ||
                      kv.first == 220 || kv.first == 217 || kv.first == 219 ||
                      kv.first == 226 || kv.first == 227);
    snprintf(val, sizeof(val), "%d",
             sgn && kv.second > 32767 ? (int)kv.second - 65536 : (int)kv.second);
    mqtt_test_deliver(topic, val, retained);
  }
}
}  // namespace

TEST(LgResuMirror, ServesTheRealBatterysRegistersVerbatim) {
  current_time = 100000;
  mqtt_test_reset_subscriptions();
  ReplayInverter inv;
  inv.setup();  // enables mirror on the default topic
  ASSERT_TRUE(inv.mirror_enabled()) << "mirror must be on by default for this protocol";
  ASSERT_EQ(mqtt_test_subscription_count(), 1u);

  // Collect the real battery's register values, then publish them as the Pi would.
  // NOTE: the corpus spans 60 s and dynamic registers change within it, so the check is
  // "what we published comes back", not "every historical frame replays" -- comparing a
  // single snapshot against 184 different moments would fail on live telemetry alone.
  std::map<uint16_t, uint16_t> fed;
  for (auto& e : load_corpus()) {
    if (e.fn != 0x03) continue;
    auto p = payload(e.rsp);
    if ((int)p.size() < 3 + e.count * 2) continue;
    for (uint16_t i = 0; i < e.count; i++)
      fed[e.start + i] = (uint16_t)(p[3 + i * 2] << 8 | p[4 + i * 2]);
  }
  ASSERT_GE(fed.size(), 60u) << "expected the corpus to cover most of the read contract";

  feed_from_corpus();
  inv.update_values();

  int checked = 0, no_datalayer_equivalent = 0;
  for (auto& kv : fed) {
    const uint16_t reg = kv.first;
    if (reg == 201) continue;  // the inverter's own write wins; see apply_mirror()
    ModbusMessage req(0x0F, 0x03, reg, uint16_t{1});
    ModbusMessage rsp = inv.FC03(req);
    auto got = bytes_of(rsp);
    ASSERT_EQ(got.size(), 5u) << "reg " << reg;
    EXPECT_EQ((uint16_t)(got[3] << 8 | got[4]), kv.second)
        << "mirrored register " << reg << " does not match the real battery";
    checked++;
    // Registers with no datalayer equivalent -- the reason mirroring exists at all.
    if (reg == 222 || reg == 223 || reg == 224 || reg == 228 || reg == 230 || reg == 234 ||
        reg == 141 || reg == 142)
      no_datalayer_equivalent++;
  }
  EXPECT_GT(checked, 60);
  EXPECT_GE(no_datalayer_equivalent, 6)
      << "mirroring must carry registers the datalayer cannot express";
}

TEST(LgResuMirror, NegativeValuesSurviveTheRoundTrip) {
  current_time = 200000;
  mqtt_test_reset_subscriptions();
  ReplayInverter inv;
  inv.setup();
  ASSERT_TRUE(inv.enable_mirror("lg/master/sensor/+/state"));

  mqtt_test_deliver("lg/master/sensor/lg_203/state", "-1002");  // discharging
  mqtt_test_deliver("lg/master/sensor/lg_216/state", "-67");
  mqtt_test_deliver("lg/master/sensor/lg_218/state", "-1");
  inv.update_values();

  EXPECT_EQ(inv.mbPV[203], 64534) << "-1002 W must land as its two's-complement word";
  EXPECT_EQ(inv.mbPV[216], 65469);
  EXPECT_EQ(inv.mbPV[218], 65535);
}

TEST(LgResuMirror, IgnoresRegistersOutsideTheReadContract) {
  current_time = 300000;
  mqtt_test_reset_subscriptions();
  ReplayInverter inv;
  inv.setup();
  ASSERT_TRUE(inv.enable_mirror("lg/master/sensor/+/state"));

  // The 1100 config block is written BY the inverter; mirroring it back would fight the
  // 1101 -> 201 mirror and could re-assert a stale mode.
  mqtt_test_deliver("lg/master/sensor/lg_1101/state", "1");
  // Also publish a 201 the real pack might be reporting; the inverter's own write must win.
  mqtt_test_deliver("lg/master/sensor/lg_201/state", "1");
  inv.update_values();
  EXPECT_EQ(inv.mbPV[201], 3)
      << "201 is the read side of 1101, which the INVERTER writes -- its own command wins "
         "over anything the real pack reports, or it would command standby and see active";
  EXPECT_EQ(inv.mbPV.count(1101), 0u) << "the 1100 config block must not be mirrored back";
}

TEST(LgResuMirror, StaleDataForcesPowerLimitsToZero) {
  current_time = 400000;
  mqtt_test_reset_subscriptions();
  ReplayInverter inv;
  inv.setup();
  ASSERT_TRUE(inv.enable_mirror("lg/master/sensor/+/state"));

  feed_from_corpus();
  inv.update_values();
  ASSERT_TRUE(inv.mirror_is_fresh());
  ASSERT_GT(inv.mbPV[213], 0) << "precondition: fresh data carries a real charge limit";

  // Advance past the staleness window without delivering anything further.
  current_time += LgResuPrimeModbusInverter::kMirrorStaleMs + 1000;
  inv.update_values();

  EXPECT_FALSE(inv.mirror_is_fresh());
  EXPECT_EQ(inv.mbPV[213], 0) << "stale data must not leave a live charge limit";
  EXPECT_EQ(inv.mbPV[229], 0) << "nor a live discharge limit";
  EXPECT_EQ(inv.mbPV[226], 0);
  EXPECT_EQ(inv.mbPV[211], 0);
  EXPECT_EQ(inv.mbPV[214], 0);

  // Voltage keeps its last value: the pack stays visibly present, it simply will not move
  // power. Going silent was the alternative and is documented in apply_mirror() as the
  // weaker choice. SOC does NOT keep its last value -- it is forced to 10.1 %, below the
  // Delta's observed 11 % discharge cut and above the pack's own 8 % protection limit,
  // because the inverter-side floor is the one lever the Delta is proven to obey.
  EXPECT_EQ(inv.mbPV[221], 101) << "stale SOC must read 10.1 %, not the last live value";
  ASSERT_GT(inv.mbPV[205], 0) << "precondition: the corpus carries a full-capacity figure";
  EXPECT_EQ(inv.mbPV[206], (uint16_t)((uint32_t)inv.mbPV[205] * 101 / 1000))
      << "206/205 is how a SolarEdge derives SOC; it must agree with 221";
}

// --- retained-message liveness --------------------------------------------
//
// The Pi publishes every register with retain=True, and BE re-subscribes on every MQTT
// reconnect -- so the broker replays the whole set to it. Arrival must not be mistaken for
// freshness, or a flapping link with a dead publisher looks permanently alive. But retained
// data still has to SEED a cold start: an unpopulated mbPV answers reads with 0, and a
// battery reporting 0 V is a worse lie than a five-minute-old voltage.

TEST(LgResuMirror, RetainedSeedMakesAColdStartServePlausibleValues) {
  current_time = 500000;
  mqtt_test_reset_subscriptions();
  ReplayInverter inv;
  inv.setup();
  ASSERT_TRUE(inv.enable_mirror("lg/master/sensor/+/state"));

  // Nothing live has ever arrived -- only the broker's retained backlog.
  feed_from_corpus(1000, /*retained=*/true);
  inv.update_values();

  EXPECT_TRUE(inv.mirror_is_fresh())
      << "the first retained batch must seed the clock, or a cold start serves 0 V";
  EXPECT_GT(inv.mbPV[202], 3000) << "and it must carry the pack's real bus voltage";
  EXPECT_GT(inv.mbPV[213], 0) << "with live limits, not the degraded state";
}

TEST(LgResuMirror, RetainedReplayDoesNotRestartTheClock) {
  current_time = 600000;
  mqtt_test_reset_subscriptions();
  ReplayInverter inv;
  inv.setup();
  ASSERT_TRUE(inv.enable_mirror("lg/master/sensor/+/state"));

  feed_from_corpus();  // live data from a healthy master
  inv.update_values();
  ASSERT_TRUE(inv.mirror_is_fresh());

  // The master dies. Later, BE's MQTT session flaps and the broker replays everything it
  // holds retained -- values that have not been refreshed since the last live message.
  current_time += LgResuPrimeModbusInverter::kMirrorStaleMs - 1000;
  feed_from_corpus(1000, /*retained=*/true);
  inv.update_values();
  EXPECT_TRUE(inv.mirror_is_fresh()) << "still inside the window measured from the live data";

  current_time += 2000;  // now past it -- and the replay must not have deferred this
  inv.update_values();
  EXPECT_FALSE(inv.mirror_is_fresh())
      << "a retained replay is not evidence the master is alive";
  EXPECT_EQ(inv.mbPV[221], 101) << "so the degraded state must engage on schedule";
  EXPECT_EQ(inv.mbPV[213], 0);
}

TEST(LgResuMirror, RepeatedRetainedReplaysCannotHoldOffStalenessForever) {
  // The pathological case the old code hit: BE reboots while the master is already dead,
  // so it never sees a live message at all, and a flapping link re-delivers the retained
  // backlog every couple of minutes. Seeding once is what bounds this.
  current_time = 700000;
  mqtt_test_reset_subscriptions();
  ReplayInverter inv;
  inv.setup();
  ASSERT_TRUE(inv.enable_mirror("lg/master/sensor/+/state"));

  feed_from_corpus(1000, /*retained=*/true);
  inv.update_values();
  ASSERT_TRUE(inv.mirror_is_fresh());

  for (int flap = 0; flap < 4; flap++) {
    current_time += LgResuPrimeModbusInverter::kMirrorStaleMs / 2;
    feed_from_corpus(1000, /*retained=*/true);
    inv.update_values();
  }

  EXPECT_FALSE(inv.mirror_is_fresh())
      << "four reconnects must not add up to indefinite freshness";
  EXPECT_EQ(inv.mbPV[221], 101);
}

// --- 201 state model -------------------------------------------------------
//
// Measured against the real pack 2026-08-27, with the inverter off and the 12 V enable
// LOW: all eight boot writes echoed, and 201 stayed at 1 despite 1101 <- 3. So 201 is not
// a plain mirror of 1101 -- readiness comes from the enable line.

TEST(LgResuState, StandbyWhileTheInverterHasNotEnabledUs) {
  ReplayInverter inv;
  inv.setup();
  datalayer.system.status.inverter_allows_contactor_closing = false;

  ModbusMessage w(0x0F, 0x06, uint16_t{1101}, uint16_t{3});
  inv.FC06(w);           // "go active" -- accepted and echoed, as the real pack does
  inv.update_values();

  EXPECT_EQ(inv.mbPV[201], 1)
      << "1101 <- 3 must not make the pack active while the inverter has not enabled it";
}

TEST(LgResuState, GoesActiveOnceEnabledWithoutBeingCommanded) {
  ReplayInverter inv;
  inv.setup();
  datalayer.system.status.inverter_allows_contactor_closing = true;
  inv.update_values();

  // The real pack reads 201 = 3 at +2.38 s on a cold start, BEFORE the inverter writes
  // anything -- it goes active on its own once enabled.
  EXPECT_EQ(inv.mbPV[201], 3) << "an enabled pack reports active without being told to";
}

TEST(LgResuState, StandbyCommandLatchesAndIsReleasedByGoActive) {
  ReplayInverter inv;
  inv.setup();
  datalayer.system.status.inverter_allows_contactor_closing = true;
  inv.update_values();
  ASSERT_EQ(inv.mbPV[201], 3);

  ModbusMessage stop(0x0F, 0x06, uint16_t{1101}, uint16_t{1});
  inv.FC06(stop);
  inv.update_values();
  EXPECT_EQ(inv.mbPV[201], 1) << "1101 <- 1 forces standby; proven on the real pack 4 times";

  // Still enabled, but standby is latched: it must not drift back on its own.
  inv.update_values();
  EXPECT_EQ(inv.mbPV[201], 1);

  ModbusMessage go(0x0F, 0x06, uint16_t{1101}, uint16_t{3});
  inv.FC06(go);
  inv.update_values();
  EXPECT_EQ(inv.mbPV[201], 3) << "1101 <- 3 releases the latch";
}

TEST(LgResuState, LosingEnableDropsBackToStandby) {
  ReplayInverter inv;
  inv.setup();
  datalayer.system.status.inverter_allows_contactor_closing = true;
  inv.update_values();
  ASSERT_EQ(inv.mbPV[201], 3);

  // The real enable falls 12.3 s after the inverter stops polling, as its 12 V rail
  // bleeds down. A pack cannot remain active through that.
  datalayer.system.status.inverter_allows_contactor_closing = false;
  inv.update_values();
  EXPECT_EQ(inv.mbPV[201], 1);
}
