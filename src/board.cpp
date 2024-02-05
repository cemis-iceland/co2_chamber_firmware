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

void power_on() {
  const int channel = 1;
  ledcSetup(channel, 1E6, 6);
  ledcAttachPin(PIN_PWR_EN, channel);
  for (int duty = 0; duty < 64; duty++) {
    ledcWrite(channel, duty);
    delayMicroseconds(1000);
  }
  ledcDetachPin(PIN_PWR_EN);
  digitalWrite(PIN_PWR_EN, HIGH);
}

void power_off() { digitalWrite(PIN_PWR_EN, LOW); }

void open_valves() {
  //Valve 1
  digitalWrite(PIN_VALVE_1_REV, HIGH);
  vTaskDelay(50 / portTICK_PERIOD_MS);
  digitalWrite(PIN_VALVE_1_FWD, HIGH);
  vTaskDelay(50 / portTICK_PERIOD_MS);
  digitalWrite(PIN_VALVE_1_FWD, LOW);
  digitalWrite(PIN_VALVE_1_REV, LOW);
  //Valve 2
  digitalWrite(PIN_VALVE_2_REV, HIGH);
  vTaskDelay(50 / portTICK_PERIOD_MS);
  digitalWrite(PIN_VALVE_2_FWD, HIGH);
  vTaskDelay(50 / portTICK_PERIOD_MS);
  digitalWrite(PIN_VALVE_2_FWD, LOW);
  digitalWrite(PIN_VALVE_2_REV, LOW);
}

void close_valves() {
  //Valve 1
  digitalWrite(PIN_VALVE_1_FWD, HIGH);
  vTaskDelay(50 / portTICK_PERIOD_MS);
  digitalWrite(PIN_VALVE_1_REV, HIGH);
  vTaskDelay(50 / portTICK_PERIOD_MS);
  digitalWrite(PIN_VALVE_1_REV, LOW);
  digitalWrite(PIN_VALVE_1_FWD, LOW);
  //Valve 2
  digitalWrite(PIN_VALVE_2_FWD, HIGH);
  vTaskDelay(50 / portTICK_PERIOD_MS);
  digitalWrite(PIN_VALVE_2_REV, HIGH);
  vTaskDelay(50 / portTICK_PERIOD_MS);
  digitalWrite(PIN_VALVE_2_REV, LOW);
  digitalWrite(PIN_VALVE_2_FWD, LOW);
}

void fan_on() { digitalWrite(PIN_FAN, HIGH); }
void fan_off() { digitalWrite(PIN_FAN, LOW); }

} // namespace board