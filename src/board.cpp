#include "board.h"
#include "pin_assignments.h"
#include <Arduino.h>

namespace board {
void setup_gpio() {
  pinMode(PIN_PWR_EN, OUTPUT);
  pinMode(PIN_STATUS_LED, OUTPUT);
  pinMode(PIN_FAN, OUTPUT);
  pinMode(PIN_VALVE_1_FWD, OUTPUT);
  pinMode(PIN_VALVE_1_REV, OUTPUT);
  pinMode(PIN_VALVE_2_FWD, OUTPUT);
  pinMode(PIN_VALVE_2_REV, OUTPUT);

  digitalWrite(PIN_PWR_EN, LOW);
  digitalWrite(PIN_STATUS_LED, LOW);
  digitalWrite(PIN_FAN, LOW);
  digitalWrite(PIN_VALVE_1_FWD, LOW);
  digitalWrite(PIN_VALVE_1_REV, LOW);
  digitalWrite(PIN_VALVE_2_FWD, LOW);
  digitalWrite(PIN_VALVE_2_REV, LOW);
}

void power_on() { digitalWrite(PIN_PWR_EN, HIGH); }
void power_off() { digitalWrite(PIN_PWR_EN, LOW); }

void open_valves() {
  digitalWrite(PIN_VALVE_1_REV, HIGH);
  vTaskDelay(200 / portTICK_PERIOD_MS);
  digitalWrite(PIN_VALVE_1_REV, LOW);
  vTaskDelay(200 / portTICK_PERIOD_MS);
  digitalWrite(PIN_VALVE_2_REV, HIGH);
  vTaskDelay(200 / portTICK_PERIOD_MS);
  digitalWrite(PIN_VALVE_2_REV, LOW);
}

void close_valves() {
  digitalWrite(PIN_VALVE_1_FWD, HIGH);
  vTaskDelay(200 / portTICK_PERIOD_MS);
  digitalWrite(PIN_VALVE_1_FWD, LOW);
  vTaskDelay(200 / portTICK_PERIOD_MS);
  digitalWrite(PIN_VALVE_2_FWD, HIGH);
  vTaskDelay(200 / portTICK_PERIOD_MS);
  digitalWrite(PIN_VALVE_2_FWD, LOW);
}

void fan_on() { digitalWrite(PIN_FAN, HIGH); }
void fan_off() { digitalWrite(PIN_FAN, LOW); }

} // namespace board