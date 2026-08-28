/**
 * MQTT add-on for the battery emulator
 * 
 * Usage:
 * 
 * Subscription - Add topics to MQTT_SUBSCRIPTIONS in USER_SETTINGS.h and handle the messages in mqtt.cpp:callback()
 * 
 * Publishing - See example in mqtt.cpp:publish_values() for constructing the payload
 * 
 * Home assistant - See below for an example, and the official documentation is quite good (https://www.home-assistant.io/integrations/sensor.mqtt/)
 * in configuration.yaml:
 * mqtt: !include mqtt.yaml
 * 
 * in mqtt.yaml:
 * sensor:
 *   - name: "Cell max"
 *       state_topic: "battery/info"
 *       unit_of_measurement: "mV"
 *       value_template: "{{ value_json.cell_max_voltage | int }}"
 *   - name: "Cell min"
 *       state_topic: "battery/info"
 *       unit_of_measurement: "mV"
 *       value_template: "{{ value_json.cell_min_voltage | int }}"
 *   - name: "Temperature max"
 *       state_topic: "battery/info"
 *       unit_of_measurement: "C"
 *       value_template: "{{ value_json.temperature_max | float }}"
 *   - name: "Temperature min"
 *       state_topic: "battery/info"
 *       unit_of_measurement: "C"
 *       value_template: "{{ value_json.temperature_min | float }}"
 */

#ifndef __MQTT_H__
#define __MQTT_H__

#include <Arduino.h>
#include <string>
#include <vector>

#define MQTT_MSG_BUFFER_SIZE (4096)

extern const char* version_number;  // The current software version, used for mqtt

extern bool mqtt_enabled;
extern bool mqtt_transmit_all_cellvoltages;
extern bool mqtt_publish_heap_metrics;
extern uint16_t mqtt_timeout_ms;
extern uint16_t mqtt_publish_interval_ms;
extern bool ha_autodiscovery_enabled;
extern std::string ha_autodiscovery_topic;
extern std::string mqtt_server;
extern std::string mqtt_user;
extern std::string mqtt_password;
extern int mqtt_port;
extern const char* mqtt_topic_name;
extern const char* mqtt_object_id_prefix;
extern const char* mqtt_device_name;
extern const char* ha_device_id;

extern char mqtt_msg[MQTT_MSG_BUFFER_SIZE];

uint32_t mqtt_firmware_signature(void);
bool init_mqtt(void);
void mqtt_client_loop(void);
bool mqtt_publish(const char* topic, const char* mqtt_msg, bool retain);

/* Subscribe to a topic outside the emulator's own tree.
 *
 * The emulator subscribes to "<hostname>/command/+" for its own buttons. Modules that
 * need to CONSUME data -- an inverter protocol mirroring a real battery's registers
 * published by another device, say -- had no way to ask for a topic.
 *
 * Register before init_mqtt(); subscriptions are (re)issued on every broker connect, so
 * they survive reconnects. The handler runs on the MQTT task: keep it short, and do not
 * block or touch the shared mqtt_msg / shared_doc buffers, which are single-threaded by
 * contract.
 *
 * `filter` may contain MQTT wildcards (+ and #) and must remain valid for the lifetime
 * of the program (a string literal is the intended use).
 */
/* `retained` is the broker's retained flag for this message.
 *
 * It matters to any handler that uses arrival as a LIVENESS signal: the broker replays
 * every retained topic to a late subscriber, so a reconnect delivers a full set of
 * messages whose data may be arbitrarily old. A handler that stamps a clock on arrival
 * will read that as "the publisher is alive" when it may be long dead.
 */
typedef void (*MqttTopicHandler)(const char* topic, int topic_len, const char* data, int data_len,
                                 bool retained);
bool mqtt_register_subscription(const char* filter, MqttTopicHandler handler);

#endif
