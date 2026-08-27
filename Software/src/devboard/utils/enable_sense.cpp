#include "enable_sense.h"

#include <Arduino.h>
#include <WiFi.h>
#include <stdio.h>

#include "../../datalayer/datalayer.h"
#include "../hal/hal.h"
#include "../mqtt/mqtt.h"
#include "logging.h"

bool enable_sense_active_high = true;
bool enable_sense_drive_datalayer = false;

namespace {

// A 12 V line off an inverter, sensed through a divider, is electrically quiet but the
// inverter's own switching is not. Require the reading to hold for DEBOUNCE_MS before
// believing it. At the 1 ms core_loop cadence this is 25 consecutive agreeing samples.
constexpr uint32_t DEBOUNCE_MS = 25;

// Republish the retained state periodically so a static value cannot be mistaken for a
// dead publisher. (The sniffer README documents exactly that failure mode: a register that
// never changes is indistinguishable from one that stopped being read.)
constexpr uint32_t HEARTBEAT_MS = 30000;

// Edges are rare — a handful per power cycle — but they are the whole point of this module,
// so they are queued rather than dropped if MQTT happens to be down when one occurs.
constexpr size_t EDGE_QUEUE_LEN = 16;

// Publishes fail whenever the MQTT client is absent (init_mqtt() aborts when no battery is
// selected), the broker is unreachable, or the outbox is full. This loop runs at the 1 ms
// core_loop cadence, so retrying on every pass floods the ESP-IDF log with
// "mqtt_client: Client was not initialized" and burns CPU. Back off instead. Found by
// running it: the first flash with no battery configured produced ~1000 lines/second.
constexpr uint32_t PUBLISH_RETRY_MS = 2000;

struct Edge {
  uint32_t t_ms;
  bool state;
};

gpio_num_t pin = GPIO_NUM_NC;
bool initialised = false;

bool stable_state = false;
bool have_stable = false;
bool candidate = false;
uint32_t candidate_since = 0;

Edge queue[EDGE_QUEUE_LEN];
size_t q_head = 0, q_tail = 0;
uint32_t last_heartbeat = 0;
uint32_t next_attempt_ms = 0;  // publish backoff, see PUBLISH_RETRY_MS

char topic_state[96];
char topic_event[96];
char payload[128];

inline bool q_empty() {
  return q_head == q_tail;
}

void q_push(uint32_t t_ms, bool state) {
  size_t next = (q_head + 1) % EDGE_QUEUE_LEN;
  if (next == q_tail) {
    // Full: drop the OLDEST. The most recent edges are the ones being measured.
    q_tail = (q_tail + 1) % EDGE_QUEUE_LEN;
  }
  queue[q_head] = {t_ms, state};
  q_head = next;
}

}  // namespace

bool enable_sense_init(void) {
  if (initialised) {
    return true;
  }
  pin = esp32hal->INVERTER_CONTACTOR_ENABLE_PIN();
  if (pin == GPIO_NUM_NC) {
    logging.println("enable_sense: this board's HAL defines no INVERTER_CONTACTOR_ENABLE_PIN");
    return false;
  }
  if (!esp32hal->alloc_pins("enable sense", pin)) {
    logging.println("enable_sense: pin already allocated to another function");
    return false;
  }
  // Plain INPUT: the divider defines both levels. With an optocoupler pulling the pin down,
  // use INPUT_PULLUP here and set enable_sense_active_high = false.
  pinMode(pin, enable_sense_active_high ? INPUT : INPUT_PULLUP);

  // Topic base = the hostname, matching how mqtt.cpp builds `topic_name` in init_mqtt().
  // NOT mqtt_topic_name: that symbol is declared extern in mqtt.h and never defined
  // anywhere in the tree, so referencing it is a link error. (Upstream nit, v12.3.0.)
  const char* host = WiFi.getHostname();
  if (host == nullptr || host[0] == '\0') {
    host = "battery-emulator";
  }
  snprintf(topic_state, sizeof(topic_state), "%s/enable", host);
  snprintf(topic_event, sizeof(topic_event), "%s/enable_event", host);

  initialised = true;
  logging.printf("enable_sense: watching GPIO%d, active %s\n", (int)pin,
                 enable_sense_active_high ? "HIGH" : "LOW");
  return true;
}

bool enable_sense_state(void) {
  return have_stable && stable_state;
}

void enable_sense_loop(void) {
  if (!initialised) {
    return;
  }
  const uint32_t now = millis();

  // ---- debounce ----
  bool raw = (digitalRead(pin) == HIGH);
  if (!enable_sense_active_high) {
    raw = !raw;
  }
  if (raw != candidate) {
    candidate = raw;
    candidate_since = now;
  } else if ((now - candidate_since) >= DEBOUNCE_MS) {
    if (!have_stable || candidate != stable_state) {
      stable_state = candidate;
      have_stable = true;
      q_push(now, stable_state);
      logging.printf("enable_sense: 12 V enable -> %d at t=%lu ms\n", stable_state ? 1 : 0,
                     (unsigned long)now);
      if (enable_sense_drive_datalayer) {
        datalayer.system.status.inverter_allows_contactor_closing = stable_state;
      }
    }
  }

  // ---- publish, with backoff ----
  if (now < next_attempt_ms) {
    return;  // still backing off; edges stay queued and keep their original t_ms
  }

  // ---- drain the edge queue ----
  // mqtt_publish() wraps esp_mqtt_client_publish(), which is thread-safe. A local buffer is
  // used deliberately: the shared mqtt_msg/shared_doc in mqtt.cpp are single-threaded by
  // contract and must not be touched from core_loop.
  while (!q_empty()) {
    const Edge& e = queue[q_tail];
    // age_ms lets a consumer reconstruct the true edge time even when delivery was delayed,
    // which matters because the whole point is measuring WHEN the inverter asserts enable.
    snprintf(payload, sizeof(payload), "{\"enable\":%d,\"t_ms\":%lu,\"age_ms\":%lu}",
             e.state ? 1 : 0, (unsigned long)e.t_ms, (unsigned long)(now - e.t_ms));
    if (!mqtt_publish(topic_event, payload, false)) {
      next_attempt_ms = now + PUBLISH_RETRY_MS;  // MQTT down — keep the edge, retry later
      return;
    }
    snprintf(payload, sizeof(payload), "%d", e.state ? 1 : 0);
    mqtt_publish(topic_state, payload, true);  // retained: last known state
    q_tail = (q_tail + 1) % EDGE_QUEUE_LEN;
    last_heartbeat = now;
    next_attempt_ms = 0;
  }

  // ---- heartbeat ----
  if (have_stable && q_empty() && (now - last_heartbeat) >= HEARTBEAT_MS) {
    snprintf(payload, sizeof(payload), "%d", stable_state ? 1 : 0);
    if (mqtt_publish(topic_state, payload, true)) {
      last_heartbeat = now;
    } else {
      next_attempt_ms = now + PUBLISH_RETRY_MS;
    }
  }
}
