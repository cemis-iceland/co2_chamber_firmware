#include "SCD30_MB.hpp"
#include "meas_format.h"
#include "pin_assignments.h"
#include "configurator.h"
#include "logger.h"
#include <Arduino.h>
#include <Adafruit_BME280.h>
#include <SD.h>
#include <SPI.h>
#include <chrono>
#include <iomanip>
#include <iostream>
#include <mbcontroller.h>
#include <sstream>
#include <string>
#include <sys/time.h>
#include <json11.hpp>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <rom/rtc.h>
#include <DNSServer.h>
#ifdef ESP32
#include <AsyncTCP.h>
#include <WiFi.h>
#endif
#include "ESPAsyncWebServer.h"

#include <Preferences.h>
/*
 * Note about the Preferences library:
 * It uses the non-volatile storage partition to store the preferences, which
 * can become full. To erase it use:
 *
 * #include <nvs_flash.h>
 * void setup() {
 *  nvs_flash_erase(); // Erase NVS partition
 *  nvs_flash_init(); // Initialize the NVS partition
 * }
 */

RTC_DATA_ATTR bool resetReasonDeepSleep = false;

#define LOGFILE_PREFIX "/data/measurements_"
#define LOGFILE_POSTFIX ".csv"

unsigned long loopTimer = 0;

Preferences preferences;

void setup() {
  // Prepare GPIO
  /*
   * NOTE:
   * During sleep the pins might go into undefined states
   * This can be fixed by setting the RTC function of the pins
   * This functionality NEEDS to be added for safe behaviour
   */
  pinMode(PIN_PWR_EN, OUTPUT);
  pinMode(PIN_FAN, OUTPUT);
  pinMode(PIN_STATUS_LED, OUTPUT);
  pinMode(PIN_VALVE_1_FWD, OUTPUT);
  pinMode(PIN_VALVE_1_REV, OUTPUT);
  pinMode(PIN_VALVE_2_FWD, OUTPUT);
  pinMode(PIN_VALVE_2_REV, OUTPUT);

  digitalWrite(PIN_PWR_EN, HIGH);
  digitalWrite(PIN_FAN, LOW);
  digitalWrite(PIN_STATUS_LED, LOW);
  digitalWrite(PIN_VALVE_1_FWD, LOW);
  digitalWrite(PIN_VALVE_1_REV, LOW);
  digitalWrite(PIN_VALVE_2_FWD, LOW);
  digitalWrite(PIN_VALVE_2_REV, LOW);

  Serial1.begin(19200, SERIAL_8N1, PIN_UART_RX, PIN_UART_TX);
  Serial.begin(115200);

  // Set up SD card
  SPI.begin(PIN_SPI_SCLK, PIN_SPI_MISO, PIN_SPI_MOSI);
  log_fail("SD initialization...       ", SD.begin(PIN_SD_CSN, SPI), true);

  if (resetReasonDeepSleep) {
    // Run deep sleep wakeup code (website, write config to SD, etc)
    log_i("%s SUCCESS", "Starting from deep sleep...");
    readConfig();
    preferences.begin("hardinfo");
    serial_number = std::string(preferences.getString("serial_number").c_str());
    preferences.end();
  } else {
    // Run POR startup code (read config from SD, check battery health)
    log_i("%s SUCCESS", "Starting from POR...");

    preferences.begin("hardinfo");
    if (preferences.getType("serial_number") != PT_INVALID) {
      serial_number =
          std::string(preferences.getString("serial_number").c_str());
      if (serial_number == "0") {
        log_fail("Serial number is 0, please provide valid serial number:",
                 false, false);
        bool has_valid_serial = false;
        std::string ss = "";
        while (!has_valid_serial) {
          if (Serial.available() > 0) {
            char incomingByte = Serial.read();
            if (incomingByte == 0x0D || incomingByte == 0x0A) {
              while (Serial.available() > 0) {
                Serial.read();
              }
              serial_number = ss;
              has_valid_serial = true;
            } else {
              ss += incomingByte;
            }
          }
        }
        preferences.putString("serial_number", serial_number.c_str());
      }
    }
    preferences.end();
    readConfig();
    web_server_status = xSemaphoreCreateBinary();
    xTaskCreatePinnedToCore(web_setup_task, "web-setup", 16384, NULL, 10, &web_setup, 1);
    
    // Wait for 15 minutes or the user submits the form
    unsigned long current_millis = millis();
    while ((millis() - current_millis < 54000000) && (xSemaphoreTake(web_server_status, 0) != pdTRUE)) {
      vTaskDelay(100 / portTICK_PERIOD_MS);
    }
    vTaskDelete(web_setup);
    server.end();
    dnsServer.stop();
    WiFi.mode(WIFI_MODE_NULL);

    // Create new file for measurements
    config.logfilename = LOGFILE_PREFIX + serial_number + "_" + timestamp() + LOGFILE_POSTFIX;

    writeConfig();

    if (!SD.exists("/data")) SD.mkdir("/data");
    log_i("Creating file %s", config.logfilename.c_str());
    auto file = SD.open(config.logfilename.c_str(), FILE_WRITE);
    if (!file)
      log_fail("SD Card failed to open!", false, true); // If we don't have a file we stop
    file.println((location_notes + "$\n").c_str());
    file.println(header.c_str());
    file.close();
  }

  // Create modbus for CO2 sensor
  static auto mb = Modbus(&Serial1);

  // Configure SCD30s
  scd30 = SCD30_MB(&mb);
  scd30.set_meas_interval();
  scd30.start_cont_measurements(0x0000);

  // Set up I2C peripherals
  log_fail("I2C initialization...      ", Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL));
  log_fail("BME280 Initialization... ", bme280.begin(0x76, &Wire),
           true); // set back to true, temporarily set to false for testing

  // Set up DS18B20 temperature sensor
  soil_temp.begin();
  
  vTaskDelay(100 / portTICK_PERIOD_MS);

  enterWarmup();
  vTaskDelay(config.warmup_time * 1000 / portTICK_PERIOD_MS);
  enterPremix();
  vTaskDelay(config.premix_time * 1000 / portTICK_PERIOD_MS);
  enterValvesClosed();
  vTaskDelay(config.measurement_time * 1000 / portTICK_PERIOD_MS);
  enterPostmix();
  vTaskDelay(config.postmix_time * 1000 / portTICK_PERIOD_MS);
  enterSleep(config.sleep_duration, &resetReasonDeepSleep);
}

void loop() { log_fail("Why are we in the loop?", false, true); }