/* Host-test stand-in for the parts of mqtt.cpp other modules link against.
 *
 * The real mqtt.cpp drags in esp-mqtt and the whole client lifecycle, which the host tests
 * neither have nor want. This provides just the subscription registration, and lets a test
 * inject messages as if the broker had delivered them -- so code that CONSUMES MQTT (an
 * inverter mirroring another device's published registers) is testable without a broker.
 */
#include <string.h>

#include <string>
#include <vector>

#include "../../Software/src/devboard/mqtt/mqtt.h"
#include "../../Software/src/devboard/utils/mqtt_topic_match.h"

namespace {
struct Sub {
  std::string filter;
  MqttTopicHandler handler;
};
std::vector<Sub>& subs() {
  static std::vector<Sub> s;
  return s;
}
}  // namespace

bool mqtt_register_subscription(const char* filter, MqttTopicHandler handler) {
  if (!filter || !handler) return false;
  subs().push_back({filter, handler});
  return true;
}

bool mqtt_publish(const char* topic, const char* mqtt_msg, bool retain) {
  (void)topic;
  (void)mqtt_msg;
  (void)retain;
  return true;
}

// --- test-only helpers -----------------------------------------------------

void mqtt_test_reset_subscriptions() {
  subs().clear();
}

size_t mqtt_test_subscription_count() {
  return subs().size();
}

/** Deliver a message as the broker would, to every matching subscription.
 *  `retained` models the broker replaying a retained topic to a late subscriber.
 *  Returns how many handlers received it. */
int mqtt_test_deliver(const char* topic, const char* payload, bool retained) {
  int delivered = 0;
  for (auto& s : subs()) {
    if (mqtt_topic_matches(s.filter.c_str(), topic, (int)strlen(topic))) {
      s.handler(topic, (int)strlen(topic), payload, (int)strlen(payload), retained);
      delivered++;
    }
  }
  return delivered;
}
