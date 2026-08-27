#ifndef MQTT_TOPIC_MATCH_H
#define MQTT_TOPIC_MATCH_H

#include <string.h>

/* MQTT topic filter matching.
 *
 * Header-only and free of any MQTT client dependency, so it can be unit tested without
 * pulling in esp-mqtt. Wildcard handling is the part most worth pinning down.
 *
 *   '+'  matches exactly one topic level
 *   '#'  matches the remainder, and is only meaningful as the last character
 *
 * `topic` is NOT null-terminated in the esp-mqtt event callback, hence the explicit
 * length rather than strcmp.
 */
inline bool mqtt_topic_matches(const char* filter, const char* topic, int topic_len) {
  if (!filter || !topic) {
    return false;
  }
  int f = 0, t = 0;
  const int flen = (int)strlen(filter);
  while (f < flen && t < topic_len) {
    if (filter[f] == '#') {
      return true;
    }
    if (filter[f] == '+') {
      f++;
      while (t < topic_len && topic[t] != '/') {
        t++;
      }
    } else if (filter[f] == topic[t]) {
      f++;
      t++;
    } else {
      return false;
    }
  }
  // A trailing '#' also matches the empty remainder: "a/#" matches "a".
  if (f < flen && filter[f] == '#') {
    return true;
  }
  return f == flen && t == topic_len;
}

#endif
