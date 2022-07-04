#include "SCD30_MB.hpp"
#include "meas_format.h"
#include "pin_assignments.h"
#include "webserver.h"
#include <Adafruit_BME280.h>
#include <Arduino.h>
#include <SD.h>
#include <SPI.h>
#include <chrono>
#include <iomanip>
#include <iostream>
#include <mbcontroller.h>
#include <sstream>
#include <string>
#include <sys/time.h>
extern "C" {
#include "bootloader_random.h"
}
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

// Settings
#define CONTINUE_WITHOUT_SD false

RTC_DATA_ATTR bool resetReasonDeepSleep = false;

// Sensor instances
SCD30_MB scd30;
OneWire oneWire(PIN_TEMP_SENSOR);
DallasTemperature soil_temp(&oneWire);

Adafruit_BME280 bme280{};
auto bme_temp = bme280.getTemperatureSensor();
auto bme_pres = bme280.getPressureSensor();
auto bme_hume = bme280.getHumiditySensor();

#define CONF_FILE "/chamber_conf.json"
#define LOGFILE_PREFIX "/data/measurements_"
#define LOGFILE_POSTFIX ".csv"

unsigned long loopTimer = 0;

Webserver webserver;

Preferences preferences;

std::string serial_number = "0-0";

const char data_dir[] = "/data";

std::string location_notes;

// Create a string timestamp from the internal clock
std::string timestamp() {
  char buf[128]{0};
  time_t t;
  time(&t);
  strftime(buf, 128, "%FT%Hh%Mm%Ss", gmtime(&t));
  std::string ss = std::string{buf};
  return ss;
}

// Set time from the time submitted with the web form
void setTimeFromWeb(String time_string) {
  log_i("Time: %s", time_string.c_str());
  time_t rawtime = (time_t)(time_string.toDouble() / 1000); // Convert to seconds / 1000
  log_i("More time: %d", rawtime);
  struct timeval tv;
  struct timezone tz;
  tv.tv_sec = rawtime;
  tv.tv_usec = 0;
  tz.tz_minuteswest = 0;
  tz.tz_dsttime = 0;
  settimeofday(&tv, &tz);
}

// Create strings for logging data
const std::string delim = ","; // Csv column delimiter
const std::string sep = ".";   // Decimal seperator
const auto header = "time" + delim + "variable" + delim + "value";
std::stringstream& fmt_meas(std::string time, std::stringstream& ss,
                            std::string variable, std::string value) {
  ss << time << delim << variable << delim << value << '\n';
  return ss;
}
std::stringstream& fmt_meas(std::string time, std::stringstream& ss,
                            std::string variable, float value,
                            int precision = 8) {
  ss << time << delim << variable << delim << std::setprecision(precision)
     << value << '\n';
  return ss;
}

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

void writeConfig() {
  // Create new file for measurements
  std::string filename = CONF_FILE;

  log_i("Creating file %s", filename.c_str());

  SD.remove(filename.c_str());
  auto file = SD.open(filename.c_str(), FILE_WRITE);
  if (!file)
    log_fail("SD Card failed to open!", false,
             true); // If we don't have file we stop

  json11::Json configuration =
      json11::Json::object{{"version", (int)1},
                           {"latitude", config.latitude},
                           {"longitude", config.longitude},
                           {"warmup_time", config.warmup_time},
                           {"premix_time", config.premix_time},
                           {"measurement_time", config.measurement_time},
                           {"postmix_time", config.postmix_time},
                           {"co2_meas_interval", config.co2_meas_interval},
                           {"soil_meas_interval", config.soil_meas_interval},
                           {"sleep_duration", config.sleep_duration},
                           {"log_file_name", config.logfilename}};
  std::string configuration_str = configuration.dump();
  file.print(configuration_str.c_str());
  file.close();
}

void readConfig() {
  std::string filename = CONF_FILE;

  if (!SD.exists(filename.c_str())) {
    log_i("File %s doesn't exist, creating default file", filename.c_str());
    writeConfig();
  }

  auto file = SD.open(filename.c_str(), FILE_READ);
  if (!file)
    log_fail("SD Card failed to open!", false,
             true); // If we don't have file we stop
  std::string buf = "";
  while (file.available()) {
    buf += file.read();
  }
  std::string err;
  json11::Json configuration = json11::Json::parse(buf, err);
  int version = configuration["version"].int_value();
  if (version == 1) {
    config.latitude = configuration["latitude"].int_value();
    config.longitude = configuration["longitude"].int_value();
    config.warmup_time = configuration["warmup_time"].int_value();
    config.premix_time = configuration["premix_time"].int_value();
    config.measurement_time = configuration["measurement_time"].int_value();
    config.postmix_time = configuration["postmix_time"].int_value();
    config.co2_meas_interval = configuration["co2_meas_interval"].int_value();
    config.soil_meas_interval = configuration["soil_meas_interval"].int_value();
    config.sleep_duration = configuration["sleep_duration"].int_value();
    config.logfilename = configuration["log_file_name"].string_value();
  }
}

static TaskHandle_t measure_co2 = NULL;
static TaskHandle_t measure_soil = NULL;

static SemaphoreHandle_t SD_mutex;

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

void enterSleep(uint64_t sleepTime) {
  log_i("Entering sleep");
  vTaskDelete(measure_co2);
  vTaskDelete(measure_soil);
  resetReasonDeepSleep = true;
  digitalWrite(PIN_PWR_EN, LOW);
  //esp_sleep_pd_config(ESP_PD_DOMAIN_XTAL, ESP_PD_OPTION_OFF); // Turn off the crystal oscillator
  esp_deep_sleep((uint32_t)sleepTime * 60000000);
}

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

  // Visual indicator for reset, remove for production
  /*
  for (int i = 0; i < 3; i++) {
    digitalWrite(PIN_STATUS_LED, HIGH);
    vTaskDelay(10 / portTICK_PERIOD_MS);
    digitalWrite(PIN_STATUS_LED, LOW);
    vTaskDelay(500 / portTICK_PERIOD_MS);
  }
  */

  Serial1.begin(19200, SERIAL_8N1, PIN_UART_RX, PIN_UART_TX);
  Serial.begin(115200);

  // Set up SD card
  SPI.begin(PIN_SPI_SCLK, PIN_SPI_MISO, PIN_SPI_MOSI);
  log_fail("SD initialization...       ", SD.begin(PIN_SD_CSN, SPI), !CONTINUE_WITHOUT_SD);

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
      log_fail("SD Card failed to open!", false,
               true); // If we don't have a file we stop
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
  enterSleep(config.sleep_duration);
}

void loop() { log_fail("Why are we in the loop?", false, true); }