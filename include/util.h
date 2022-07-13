#ifndef UTIL_H20220711
#define UTIL_H20220711
#include "pin_assignments.h"
#include <Arduino.h>

// Logging for serial monitor
// TODO: add battery level check to ensure we don't kill the battery
inline bool log_fail(const char* msg, bool val, bool freeze = true) {
  if (val) {
    log_i("%s SUCCESS", msg);
  } else {
    log_e("%s FAILURE                <--- WARNING", msg);
    if (freeze) {
      for (int i = 0; i < 20; i++) {
        digitalWrite(PIN_STATUS_LED, HIGH);
        delay(50);
        digitalWrite(PIN_STATUS_LED, LOW);
        delay(200);
      }
      ESP.restart();
    }
  }
  return val;
}

// Create a string timestamp from the internal clock
inline std::string timestamp() {
  char buf[128]{0};
  time_t t;
  time(&t);
  strftime(buf, 128, "%FT%Hh%Mm%Ss", gmtime(&t));
  return std::string{buf};
}

inline std::string readLine() {
  std::string ss{};
  while (true) {
    if (Serial.available() > 0) {
      char incomingByte = Serial.read();
      Serial.write(incomingByte); // Echo
      if (incomingByte == '\r' || incomingByte == '\n') {
        // Consume any garbage
        while (Serial.available() > 0) {
          Serial.read();
        }
        return ss;
      } else if (incomingByte == '\b' && !ss.empty()) { // Backspace
        ss.pop_back();
      } else {
        ss += incomingByte;
      }
    }
  }
}
#endif // UTIL_H
