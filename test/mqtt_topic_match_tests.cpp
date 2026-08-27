#include <gtest/gtest.h>

#include <string.h>

#include "../Software/src/devboard/utils/mqtt_topic_match.h"

// Wildcard matching for MQTT topic filters. Header-only, so this needs no MQTT client.
//
// Motivation: modules can register their own subscriptions (an inverter mirroring a real
// battery's registers published by another device). Getting '+' wrong there means either
// missing every update or, worse, swallowing the emulator's own command topics before
// they reach their handler.

namespace {
bool m(const char* filter, const char* topic) {
  return mqtt_topic_matches(filter, topic, (int)strlen(topic));
}
}  // namespace

TEST(MqttTopicMatch, ExactMatch) {
  EXPECT_TRUE(m("a/b/c", "a/b/c"));
  EXPECT_FALSE(m("a/b/c", "a/b/d"));
  EXPECT_FALSE(m("a/b/c", "a/b"));
  EXPECT_FALSE(m("a/b", "a/b/c")) << "a filter must not match a longer topic";
}

TEST(MqttTopicMatch, SingleLevelWildcard) {
  EXPECT_TRUE(m("a/+/c", "a/b/c"));
  EXPECT_TRUE(m("a/+/c", "a/xyz/c"));
  EXPECT_FALSE(m("a/+/c", "a/b/d"));
  EXPECT_FALSE(m("a/+/c", "a/b/x/c")) << "'+' spans exactly one level";
  EXPECT_TRUE(m("+/b/c", "a/b/c"));
  EXPECT_TRUE(m("a/b/+", "a/b/c"));
}

TEST(MqttTopicMatch, MultiLevelWildcard) {
  EXPECT_TRUE(m("a/#", "a/b"));
  EXPECT_TRUE(m("a/#", "a/b/c/d"));
  EXPECT_TRUE(m("#", "anything/at/all"));
  EXPECT_FALSE(m("a/#", "b/c"));
}

TEST(MqttTopicMatch, TheFiltersThisProjectActuallyUses) {
  // The Pi republishes the real battery's registers as lg/master/sensor/lg_<reg>/state
  EXPECT_TRUE(m("lg/master/sensor/+/state", "lg/master/sensor/lg_201/state"));
  EXPECT_TRUE(m("lg/master/sensor/+/state", "lg/master/sensor/lg_2034/state"));
  EXPECT_FALSE(m("lg/master/sensor/+/state", "lg/master/sensor/lg_201/other"));

  // Must NOT swallow the emulator's own command tree, which is dispatched separately.
  EXPECT_FALSE(m("lg/master/sensor/+/state", "be3c24/command/pause"));
  EXPECT_TRUE(m("be3c24/command/+", "be3c24/command/pause"));
  EXPECT_FALSE(m("be3c24/command/+", "lg/master/sensor/lg_201/state"));
}

TEST(MqttTopicMatch, EdgeCases) {
  EXPECT_FALSE(m(nullptr, "a/b"));
  EXPECT_FALSE(mqtt_topic_matches("a/b", nullptr, 2));
  EXPECT_TRUE(m("", ""));
  EXPECT_FALSE(m("a", ""));
  // The esp-mqtt callback hands over a topic that is not null-terminated; matching must
  // respect the length rather than reading past it.
  const char* buf = "a/b/cXXXX";
  EXPECT_TRUE(mqtt_topic_matches("a/b/c", buf, 5));
  EXPECT_FALSE(mqtt_topic_matches("a/b/cX", buf, 5));
}
