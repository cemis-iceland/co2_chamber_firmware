#include "SCD30_MB.hpp"
#include "meas_format.h"
#include "pin_assignments.h"
#include <Adafruit_BME280.h>
#include <Arduino.h>
#include <SD.h>
#include <SPI.h>
#include <chrono>
#include <iomanip>
#include <mbcontroller.h>
#include <sstream>
#include <string>
#include <iostream>
#include <sys/time.h>
extern "C" {
  #include "bootloader_random.h"
}
#include <rom/rtc.h>
#include <json11.hpp>
#include <Preferences.h>
/* 
 * Note about the Preferences library:
 * It uses the non-volatile storage partition to store the preferences, which can become full.
 * To erase it use:
 * 
 * #include <nvs_flash.h>
 * void setup() {
 *  nvs_flash_erase(); // Erase NVS partition
 *  nvs_flash_init(); // Initialize the NVS partition
 * }
 */

/*
 * Functionality:
 * - Start and check reset reason
 * - Read contents from or create config file on SD
 * - Serve webpage
 * - Write input data from webpage into config file
 * - Start measurement cycle
 * - Check battery voltage and sleep indefinitely if voltage too low
 */

// Settings
#define CONTINUE_WITHOUT_SD false

RTC_DATA_ATTR bool resetReasonDeepSleep = false;
RTC_DATA_ATTR std::string logfilename = "temp.csv";

struct Config {
  float latitude = 64.136978;
  float longitude = -21.945821;
  int warmup_time = 10; // 300
  int premix_time = 10; // 180
  int measurement_time = 10; // 240
  int postmix_time = 10; // 180
  int co2_meas_interval = 2; // The interval at which the SCD30 takes measurements in seconds
  int air_meas_interval = 1; // The interval at which the BME280 takes measurements in seconds
  int soil_meas_interval = 6; // The interval at which the soil temp and soil moist sensors take measurements in seconds // Default 60s
  int sleep_duration = 10; //10 800; // The sleep duration in seconds
} config;

// Sensor instances
SCD30_MB scd30;

Adafruit_BME280 bme280_1{};
auto bme_temp_1 = bme280_1.getTemperatureSensor();
auto bme_pres_1 = bme280_1.getPressureSensor();
auto bme_hume_1 = bme280_1.getHumiditySensor();

#define CONF_FILE "/chamber_conf.json"
#define LOGFILE_PREFIX "/measurements_"
#define LOGFILE_POSTFIX ".csv"

unsigned long loopTimer = 0;

Preferences preferences;

std::string serial_number = "0";

// Create a string timestamp from the internal clock
std::string timestamp() {
  char buf[128]{0};
  time_t t;
  time(&t);
  strftime(buf, 128, "%FT%Hh%Mm%Ss,", gmtime(&t));
  // A little bodge here to get sub second timestamps as strftime doesn't support that
  std::string ss = std::string{buf};
  std::ostringstream mil;
  mil << millis();
  ss = ss + mil.str();
  return ss;
}

// Create strings for logging data
const std::string delim = ","; // Csv column delimiter
const std::string sep = ".";   // Decimal seperator
const auto header = "time" + delim + "variable" + delim + "value";
std::stringstream& fmt_meas(std::string time, std::stringstream& ss, std::string variable,
                            std::string value) {
  ss << time << delim << variable << delim << value << '\n';
  return ss;
}
std::stringstream& fmt_meas(std::string time, std::stringstream& ss, std::string variable,
                            float value, int precision = 8) {
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
  if (!file) log_fail("SD Card failed to open!", false, true); // If we don't have file we stop

  json11::Json configuration = json11::Json::object {
    { "version", (int)1},
    { "latitude", config.latitude },
    { "longitude", config.longitude },
    { "warmup_time", config.warmup_time },
    { "premix_time", config.premix_time },
    { "measurement_time", config.measurement_time },
    { "postmix_time", config.postmix_time },
    { "co2_meas_interval", config.co2_meas_interval },
    { "air_meas_interval", config.air_meas_interval },
    { "soil_meas_interval", config.soil_meas_interval },
    { "sleep_duration", config.sleep_duration }
  };
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
  if (!file) log_fail("SD Card failed to open!", false, true); // If we don't have file we stop
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
    config.air_meas_interval = configuration["air_meas_interval"].int_value();
    config.soil_meas_interval = configuration["soil_meas_interval"].int_value();
    config.sleep_duration = configuration["sleep_duration"].int_value();
  }
}

static TaskHandle_t measure_co2 = NULL;
static TaskHandle_t measure_air = NULL;
static TaskHandle_t measure_soil = NULL;

static SemaphoreHandle_t SD_mutex;

void measure_co2_task(void* parameter) {
  while (true) {
    std::stringstream ss{""};
    std::string time = timestamp();

    // Read sensor values from SCD30
    bool scdReady = false;
    scd30.data_ready(&scdReady);
    if (scdReady) {
      SCD30_Measurement scd30_meas;
      scd30.read_measurement(&scd30_meas);
      fmt_meas(time, ss, "scd30_co2", scd30_meas.co2);
      fmt_meas(time, ss, "scd30_temperature", scd30_meas.temperature);
      fmt_meas(time, ss, "scd30_humidity", scd30_meas.humidity_percent);
    }

    // Log data for debugging
    //log_d("%s", ss.str().c_str());

    while(xSemaphoreTake(SD_mutex, 0) != pdTRUE) {
      vTaskDelay(10 / portTICK_PERIOD_MS);
    }

    // Save data to file
    auto file = SD.open(logfilename.c_str(), FILE_APPEND);
    if (!file) {
      log_e("Failed to open file %s", logfilename.c_str());
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

void measure_air_task(void* parameter) {
  while (true) {
    std::stringstream ss{""};
    std::string time = timestamp();

    // Read BME280 environmental data
    sensors_event_t temp_1, hume_1, pres_1;
    bme_temp_1->getEvent(&temp_1);
    bme_hume_1->getEvent(&hume_1);
    bme_pres_1->getEvent(&pres_1);
    fmt_meas(time, ss, "air_temperature", temp_1.temperature);
    fmt_meas(time, ss, "air_humidity", hume_1.relative_humidity);
    fmt_meas(time, ss, "air_pressure", pres_1.pressure, 9);

    // Log data for debugging
    //log_d("%s", ss.str().c_str());

    while(xSemaphoreTake(SD_mutex, 0) != pdTRUE) {
      vTaskDelay(10 / portTICK_PERIOD_MS);
    }

    // Save data to file
    auto file = SD.open(logfilename.c_str(), FILE_APPEND);
    if (!file) {
      log_e("Failed to open file %s", logfilename.c_str());
      digitalWrite(PIN_STATUS_LED, HIGH);
    } else {
      file.print(ss.str().c_str());
      file.flush();
    }
    file.close();

    xSemaphoreGive(SD_mutex);
    vTaskDelay(config.air_meas_interval * 1000 / portTICK_PERIOD_MS);
  }
}

void measure_soil_task(void* parameter) {
  while (true) {
    std::stringstream ss{""};
    std::string time = timestamp();

    fmt_meas(time, ss, "soil_temperature", 0); // Replace 0 with actual soil measurement

    // Log data for debugging
    //log_d("%s", ss.str().c_str());

    while(xSemaphoreTake(SD_mutex, 0) != pdTRUE) {
      vTaskDelay(10 / portTICK_PERIOD_MS);
    }
    
    // Save data to file
    auto file = SD.open(logfilename.c_str(), FILE_APPEND);
    if (!file) {
      log_e("Failed to open file %s", logfilename.c_str());
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
  SD_mutex = xSemaphoreCreateMutex();
  xTaskCreatePinnedToCore(measure_co2_task, "measure_co2", 16384, NULL, 10, &measure_co2, 1);
  xTaskCreatePinnedToCore(measure_air_task, "measure_air", 16384, NULL, 10, &measure_air, 1);
  xTaskCreatePinnedToCore(measure_soil_task, "measure_soil", 16384, NULL, 10, &measure_soil, 1);
}

void enterPremix() {
  digitalWrite(PIN_FAN, HIGH);
}

void enterValvesClosed() {
  digitalWrite(PIN_VALVE_1_FWD, HIGH);
  vTaskDelay(200 / portTICK_PERIOD_MS);
  digitalWrite(PIN_VALVE_1_FWD, LOW);
  vTaskDelay(1000 / portTICK_PERIOD_MS);
  digitalWrite(PIN_VALVE_2_FWD, HIGH);
  vTaskDelay(200 / portTICK_PERIOD_MS);
  digitalWrite(PIN_VALVE_2_FWD, LOW);
}

void enterPostmix() {
  digitalWrite(PIN_VALVE_1_REV, HIGH);
  vTaskDelay(200 / portTICK_PERIOD_MS);
  digitalWrite(PIN_VALVE_1_REV, LOW);
  vTaskDelay(1000 / portTICK_PERIOD_MS);
  digitalWrite(PIN_VALVE_2_REV, HIGH);
  vTaskDelay(200 / portTICK_PERIOD_MS);
  digitalWrite(PIN_VALVE_2_REV, LOW);
}

void enterSleep(uint64_t sleepTime) {
  log_i("%s SUCCESS", "Entering sleep state...");
  vTaskDelete(measure_co2);
  vTaskDelete(measure_air);
  vTaskDelete(measure_soil);
  resetReasonDeepSleep = true;
  digitalWrite(PIN_PWR_EN, LOW);
  //esp_sleep_pd_config(ESP_PD_DOMAIN_XTAL, ESP_PD_OPTION_OFF); // Turn off the crystal oscillator
  esp_deep_sleep((uint32_t)sleepTime * 1000000);
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

  vTaskDelay(100 / portTICK_PERIOD_MS); // The SD card has issues, maybe it's too fast?

  Serial1.begin(19200, SERIAL_8N1, PIN_UART_RX, PIN_UART_TX);
  Serial.begin(115200);

  // Set up SD card
  SPI.begin(PIN_SPI_SCLK, PIN_SPI_MISO, PIN_SPI_MOSI);
  log_fail("SD initialization...       ", SD.begin(PIN_SD_CSN, SPI), !CONTINUE_WITHOUT_SD);
  Serial.println(resetReasonDeepSleep);

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
    if (preferences.getType("serial_number") != 10) {
      serial_number = std::string(preferences.getString("serial_number").c_str());
      if (serial_number == "0") {
        log_fail("Serial number is 0, please provide valid serial number:", false, false);
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
  }

  // Create modbus for CO2 sensor
  static auto mb = Modbus(&Serial1);

  vTaskDelay(300 / portTICK_PERIOD_MS);

  // Configure SCD30s
  scd30 = SCD30_MB(&mb);
  scd30.set_meas_interval();
  scd30.start_cont_measurements(0x0000);

  // Set up I2C peripherals
  log_fail("I2C initialization...      ", Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL));
  log_fail("BME280 Initialization... ", bme280_1.begin(0x76, &Wire), true); // set back to true, temporarily set to false for testing

  vTaskDelay(100/portTICK_PERIOD_MS);

  // Create new file for measurements
  logfilename = LOGFILE_PREFIX + serial_number + LOGFILE_POSTFIX; //The UID is not unique during startup, we can change that by either having the RF subsystem on or using uid(6, true);
  log_i("Creating file %s", logfilename.c_str());
  auto file = SD.open(logfilename.c_str(), FILE_WRITE);
  if (!file) log_fail("SD Card failed to open!", false, true); // If we don't have a file we stop
  file.println(header.c_str());
  file.close();

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

void loop() {
  log_fail("Why are we in the loop?", false, true);
}


/*********************************************************************/
/***************************** OLD CODE ******************************/
/*********************************************************************/


//Sets internal clock to use time from GPS
/*
void setTimeFromGPS() {
  struct tm time{};
  time.tm_year = gps_module.getYear() - 1900;
  time.tm_mon = gps_module.getMonth() - 1;
  time.tm_mday = gps_module.getDay();
  time.tm_hour = gps_module.getHour();
  time.tm_min = gps_module.getMinute();
  time.tm_sec = gps_module.getSecond();
  time_t rawtime = mktime(&time);
  struct timeval tv;
  struct timezone tz;
  tv.tv_sec = rawtime;
  tv.tv_usec = 0;
  tz.tz_minuteswest = 0;
  tz.tz_dsttime = 0;
  settimeofday(&tv, &tz);
}
*/

/*
void oldSetup() {
  log_i("Waiting for sensors to turn on...");
  vTaskDelay(3000 / portTICK_PERIOD_MS);
  log_i("Initializing sensors...");
  
  // Set up serial ports for CO2 sensors
  Serial1.begin(115200, SERIAL_8N2, PIN_UART_RX, PIN_UART_TX);  // Sensor bank 1
  //Serial2.begin(115200, SERIAL_8N2, U2_RX, U2_TX);  // Sensor bank 2

  // Create modbus for CO2 sensors
  static auto mb1 = Modbus(&Serial1);
  //static auto mb2 = Modbus(&Serial2);

  vTaskDelay(3000 / portTICK_PERIOD_MS);

  // Configure SCD30s
  scd30.set_meas_interval();
  scd30.start_cont_measurements(0x0000);

  // Set up I2C peripherals
  log_fail("I2C initialization...      ", Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL));
  log_fail("BME280 1 Initialization... ", bme280_1.begin(0x76, &Wire), true);

  vTaskDelay(100/portTICK_PERIOD_MS);

  // Set up SD card
  SPI.begin(PIN_SPI_SCLK, PIN_SPI_MISO, PIN_SPI_MOSI);
  log_fail("SD initialization...       ", SD.begin(PIN_SD_CSN, SPI), !CONTINUE_WITHOUT_SD);

  // Create new file for measurements
  filename = LOGFILE_PREFIX + uid() + LOGFILE_POSTFIX; //The UID is not unique during startup, we can change that by either having the RF subsystem on or using uid(6, true);
  log_i("Creating file %s", filename.c_str());
  auto file = SD.open(filename.c_str(), FILE_WRITE);
  if (!file) log_fail("SD Card failed to open!", false, true); // If we don't have file we stop
  file.println(header.c_str());
  file.close();
  digitalWrite(PIN_STATUS_LED, HIGH);
}
*/

/*
void oldLoop() {
  if (loopTimer > millis()) loopTimer = millis(); // If millis loops around we reset (happens after 49.7 days)

  if (millis() - loopTimer >= 1000) {
    loopTimer = millis();
    std::stringstream ss{""};

    std::string time = timestamp();

    // Read sensor values from SCD30
    bool scdReady = false;
    scd30.data_ready(&scdReady);
    if (scdReady) {
      SCD30_Measurement scd30_meas;
      scd30.read_measurement(&scd30_meas);
      fmt_meas(time, ss, "scd30_co2", scd30_meas.co2);
      fmt_meas(time, ss, "scd30_temperature", scd30_meas.temperature);
      fmt_meas(time, ss, "scd30_humidity", scd30_meas.humidity_percent);
    }

    // Read BME280 environmental data
    sensors_event_t temp_1, hume_1, pres_1;
    bme_temp_1->getEvent(&temp_1);
    bme_hume_1->getEvent(&hume_1);
    bme_pres_1->getEvent(&pres_1);
    fmt_meas(time, ss, "bme280_1_temperature", temp_1.temperature);
    fmt_meas(time, ss, "bme280_1_humidity", hume_1.relative_humidity);
    fmt_meas(time, ss, "bme280_1_pressure", pres_1.pressure, 9);

    // Log data for debugging
    //log_d("%s", ss.str().c_str());

    // Save data to file
    auto file = SD.open(filename.c_str(), FILE_APPEND);
    if (!file) {
      log_e("Failed to open file %s", filename.c_str());
      digitalWrite(PIN_STATUS_LED, HIGH);
    } else {
      file.print(ss.str().c_str());
      file.flush();
    }
    file.close();
  }
}
*/