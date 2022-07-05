#include "logger.h"
#include "configurator.h"
#include "pin_assignments.h"
#include "SCD30_MB.hpp"

#include <Arduino.h>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <SD.h>
#include <Adafruit_BME280.h>

// Logging for serial monitor
// TODO: add battery level check to ensure we don't kill the battery
inline bool log_fail(const char* msg, bool val, bool freeze) {
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
std::string timestamp() {
  char buf[128]{0};
  time_t t;
  time(&t);
  strftime(buf, 128, "%FT%Hh%Mm%Ss", gmtime(&t));
  std::string ss = std::string{buf};
  return ss;
}

// Create strings for logging data

std::stringstream& fmt_meas(std::string time, std::stringstream& ss,
                            std::string variable, std::string value) {
  ss << time << delim << variable << delim << value << '\n';
  return ss;
}
std::stringstream& fmt_meas(std::string time, std::stringstream& ss,
                            std::string variable, float value,
                            int precision) {
  ss << time << delim << variable << delim << std::setprecision(precision)
     << value << '\n';
  return ss;
}

void measure_co2_task(void* parameter) {
  while (true) {
    std::stringstream ss{""};
    std::string time = timestamp();

    // Read sensor values from SCD30
    bool scdReady = false;
    // Wait until SCD30 is ready
    while (!scdReady) {
      scd30.data_ready(&scdReady);
      vTaskDelay(100 / portTICK_PERIOD_MS);
    }
    if (scdReady) {
      SCD30_Measurement scd30_meas;
      scd30.read_measurement(&scd30_meas);
      fmt_meas(time, ss, "scd30_co2", scd30_meas.co2);
      fmt_meas(time, ss, "scd30_temperature", scd30_meas.temperature);
      fmt_meas(time, ss, "scd30_humidity", scd30_meas.humidity_percent);
    }

    // Read BME280 environmental data
    sensors_event_t temp_1, hume_1, pres_1;

    bme_temp->getEvent(&temp_1);
    bme_hume->getEvent(&hume_1);
    bme_pres->getEvent(&pres_1);
    fmt_meas(time, ss, "air_temperature", temp_1.temperature);
    fmt_meas(time, ss, "air_humidity", hume_1.relative_humidity);
    fmt_meas(time, ss, "air_pressure", pres_1.pressure, 9);

    // Log data for debugging
    // log_d("%s", ss.str().c_str());

    while (xSemaphoreTake(SD_mutex, 0) != pdTRUE) {
      vTaskDelay(10 / portTICK_PERIOD_MS);
    }

    // Save data to file
    auto file = SD.open(config.logfilename.c_str(), FILE_APPEND);
    if (!file) {
      log_e("Failed to open file %s", config.logfilename.c_str());
      digitalWrite(PIN_STATUS_LED, HIGH);
    } else {
      file.print(ss.str().c_str());
      file.flush();
    }
    file.close();

    xSemaphoreGive(SD_mutex);
    vTaskDelay(config.co2_meas_interval * 1000 / portTICK_PERIOD_MS);
  }
}

void measure_soil_task(void* parameter) {
  while (true) {
    std::stringstream ss{""};
    std::string time = timestamp();
    soil_temp.requestTemperatures();
    float temperature = soil_temp.getTempCByIndex(0);
    int soil_moisture = analogRead(PIN_MOIST_SENSOR);

    fmt_meas(time, ss, "soil_temperature",
             temperature, 5);
    fmt_meas(time, ss, "soil_moisture", soil_moisture);

    // Log data for debugging
    // log_d("%s", ss.str().c_str());

    while (xSemaphoreTake(SD_mutex, 0) != pdTRUE) {
      vTaskDelay(10 / portTICK_PERIOD_MS);
    }

    // Save data to file
    auto file = SD.open(config.logfilename.c_str(), FILE_APPEND);
    if (!file) {
      log_e("Failed to open file %s", config.logfilename.c_str());
      digitalWrite(PIN_STATUS_LED, HIGH);
    } else {
      file.print(ss.str().c_str());
      file.flush();
    }
    file.close();

    xSemaphoreGive(SD_mutex);
    vTaskDelay(config.soil_meas_interval * 1000 / portTICK_PERIOD_MS);
  }
}

void enterWarmup() {
  log_i("Entering warmup");
  SD_mutex = xSemaphoreCreateMutex();
  xTaskCreatePinnedToCore(measure_co2_task, "measure_co2", 16384, NULL, 10, &measure_co2, 1);
  xTaskCreatePinnedToCore(measure_soil_task, "measure_soil", 16384, NULL, 10, &measure_soil, 1);
}

void enterPremix() {
  log_i("Entering premix");
  digitalWrite(PIN_FAN, HIGH);
}

void enterValvesClosed() {
  log_i("Entering valves closed");
  digitalWrite(PIN_VALVE_1_FWD, HIGH);
  vTaskDelay(200 / portTICK_PERIOD_MS);
  digitalWrite(PIN_VALVE_1_FWD, LOW);
  vTaskDelay(200 / portTICK_PERIOD_MS);
  digitalWrite(PIN_VALVE_2_FWD, HIGH);
  vTaskDelay(200 / portTICK_PERIOD_MS);
  digitalWrite(PIN_VALVE_2_FWD, LOW);
}

void enterPostmix() {
  log_i("Entering postmix");
  digitalWrite(PIN_VALVE_1_REV, HIGH);
  vTaskDelay(200 / portTICK_PERIOD_MS);
  digitalWrite(PIN_VALVE_1_REV, LOW);
  vTaskDelay(200 / portTICK_PERIOD_MS);
  digitalWrite(PIN_VALVE_2_REV, HIGH);
  vTaskDelay(200 / portTICK_PERIOD_MS);
  digitalWrite(PIN_VALVE_2_REV, LOW);
}

void enterSleep(uint64_t sleepTime, bool* resetReason) {
  log_i("Entering sleep");
  vTaskDelete(measure_co2);
  vTaskDelete(measure_soil);
  *resetReason = true;
  digitalWrite(PIN_PWR_EN, LOW);
  //esp_sleep_pd_config(ESP_PD_DOMAIN_XTAL, ESP_PD_OPTION_OFF); // Turn off the crystal oscillator
  esp_deep_sleep((uint32_t)sleepTime * 60000000);
}