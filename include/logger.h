#include "SCD30_MB.hpp"
#include "pin_assignments.h"
#include <Arduino.h>
#include <iostream>
#include <sstream>
#include <string>
#include <SD.h>
#include <Adafruit_BME280.h>
#include <OneWire.h>
#include <DallasTemperature.h>

static TaskHandle_t measure_co2 = NULL;
static TaskHandle_t measure_soil = NULL;

static SemaphoreHandle_t SD_mutex;

const std::string delim = ","; // Csv column delimiter
const std::string sep = ".";   // Decimal seperator
const auto header = "time" + delim + "variable" + delim + "value";

// Sensor instances
SCD30_MB scd30;
OneWire oneWire(PIN_TEMP_SENSOR);
DallasTemperature soil_temp(&oneWire);

Adafruit_BME280 bme280{};
auto bme_temp = bme280.getTemperatureSensor();
auto bme_pres = bme280.getPressureSensor();
auto bme_hume = bme280.getHumiditySensor();

inline bool log_fail(const char* msg, bool val, bool freeze = true);
std::string timestamp();

std::stringstream& fmt_meas(std::string time, std::stringstream& ss,
                            std::string variable, std::string value);

std::stringstream& fmt_meas(std::string time, std::stringstream& ss,
                            std::string variable, float value,
                            int precision = 8);

void measure_co2_task(void* parameter);
void measure_soil_task(void* parameter);
void enterWarmup();
void enterPremix();
void enterValvesClosed();
void enterPostmix();
void enterSleep(uint64_t sleepTime, bool* resetReason);