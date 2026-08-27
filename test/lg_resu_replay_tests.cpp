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
