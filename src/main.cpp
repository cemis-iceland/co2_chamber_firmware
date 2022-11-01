#include "SCD30_MB.hpp"
#include "board.h"
#include "config.h"
#include "pin_assignments.h"
#include "util.h"
#include "webserver.h"

#include <Adafruit_BME280.h>
#include <Arduino.h>
#include <DallasTemperature.h>
#include <OneWire.h>
#include <SD.h>
#include <SPI.h>
#include <iomanip>
#include <sstream>
#include <string>
#include <sys/time.h>

/*
 * Functionality:
 * - Start and check reset reason
 * - Read contents from or create config in non volatile storage
 * - Serve webpage
 * - Write input data from webpage into config
 * - Start measurement cycle
 */

// Settings
#define CONTINUE_WITHOUT_SD false
#define LOGFILE_PREFIX "/data/measurements_"
#define LOGFILE_POSTFIX ".csv"

#define DEBUG_CLEAR_NVS false // Set to true to clear config storage on boot

// Globals
Config config;
RTC_DATA_ATTR bool experimentOngoing = false;

// Sensor instances

// Create strings for logging data in long format
// about long format data: https://www.statology.org/long-vs-wide-data/
const std::string delim = ","; // Csv column delimiter
const std::string sep = ".";   // Decimal seperator
const auto header = "time" + delim + "variable" + delim + "value";
/** Logs an integer to ss in long format (time, variable, value) */
std::stringstream& fmt_meas(std::string time, std::stringstream& ss,
                            std::string variable, std::string value) {
  ss << time << delim << variable << delim << value << '\n';
  return ss;
}
/** Logs a float to ss in long format (time, variable, value).
 * Optionally specify the number of digits to use with the precision param. */
std::stringstream& fmt_meas(std::string time, std::stringstream& ss,
                            std::string variable, float value,
                            int precision = 8) {
  ss << time << delim << variable << delim << std::setprecision(precision)
     << value << '\n';
  return ss;
}

static TaskHandle_t measure_co2 = NULL;
static TaskHandle_t measure_soil = NULL;

static SemaphoreHandle_t SD_mutex;

void measure_co2_task(void* parameter) {
  // Set up SCD30 CO2 sensor
  Serial1.begin(19200, SERIAL_8N1, PIN_UART_RX, PIN_UART_TX);
  auto mb = Modbus(&Serial1);
  SCD30_MB scd30;
  scd30 = SCD30_MB(&mb);
  scd30.set_meas_interval(config.co2_interval);
  scd30.start_cont_measurements(0x0000);

  // Set up BME280 temperature/pressure/humidity sensor
  Adafruit_BME280 bme280{};
  auto bme_temp = bme280.getTemperatureSensor();
  auto bme_pres = bme280.getPressureSensor();
  auto bme_hume = bme280.getHumiditySensor();
  log_fail("I2C initialization...", Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL));
  log_fail("BME280 Initialization...", bme280.begin(0x76, &Wire), true);

  // Measure
  while (true) {
    std::stringstream ss{""};
    std::string time = timestamp();

    // Make sure SCD30 has fresh data (only an issue at meas rate > 2/s)
    bool scdReady = false;
    while (!scdReady) {
      scd30.data_ready(&scdReady);
      vTaskDelay(10 / portTICK_PERIOD_MS);
    }

    // Read sensor values from SCD30 and add them into ss
    SCD30_Measurement scd30_meas;
    scd30.read_measurement(&scd30_meas);
    fmt_meas(time, ss, "scd30_co2", scd30_meas.co2);
    fmt_meas(time, ss, "scd30_temperature", scd30_meas.temperature);
    fmt_meas(time, ss, "scd30_humidity", scd30_meas.humidity_percent);

    // Read BME280 environmental data and add it into ss
    sensors_event_t temp_1, hume_1, pres_1;
    bme_temp->getEvent(&temp_1);
    bme_hume->getEvent(&hume_1);
    bme_pres->getEvent(&pres_1);
    fmt_meas(time, ss, "air_temperature", temp_1.temperature);
    fmt_meas(time, ss, "air_humidity", hume_1.relative_humidity);
    fmt_meas(time, ss, "air_pressure", pres_1.pressure, 9);

    // Save data to file
    xSemaphoreTake(SD_mutex, portMAX_DELAY);
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

    vTaskDelay(config.co2_interval * 1000 / portTICK_PERIOD_MS);
  }
}

/** Task that measures soil temperature and moisture at a set interval */
void measure_soil_task(void* parameter) {
  // Set up sensors
  OneWire oneWire(PIN_TEMP_SENSOR);
  DallasTemperature soil_temp(&oneWire);
  soil_temp.begin();

  while (true) {
    std::stringstream ss{""};
    std::string time = timestamp();

    // Measure soil temperature
    soil_temp.requestTemperatures();
    float temperature = soil_temp.getTempCByIndex(0);

    // Measure soil moisture
    int soil_moisture = analogRead(PIN_MOIST_SENSOR);

    // Format measurements
    fmt_meas(time, ss, "soil_temperature", temperature, 5);
    fmt_meas(time, ss, "soil_moisture", soil_moisture);

    // Save data to file
    xSemaphoreTake(SD_mutex, portMAX_DELAY);
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

    vTaskDelay(config.soil_interval * 1000 / portTICK_PERIOD_MS);
  }
}

void enterWarmup() {
  log_i("Entering warmup");
  SD_mutex = xSemaphoreCreateMutex();
  xTaskCreatePinnedToCore(measure_co2_task, "measure_co2", 16384, NULL, 10,
                          &measure_co2, 1);
  xTaskCreatePinnedToCore(measure_soil_task, "measure_soil", 16384, NULL, 10,
                          &measure_soil, 1);
}

void enterPremix() {
  log_i("Entering premix");
  board::fan_on();
}

void enterValvesClosed() {
  log_i("Entering valves closed");
  board::close_valves();
}

void enterPostmix() {
  log_i("Entering postmix");
  board::open_valves();
}

void enterSleep(uint64_t sleepTime) {
  log_i("Entering sleep");
  vTaskDelete(measure_co2);
  vTaskDelete(measure_soil);
  experimentOngoing = true;
  board::power_off();
  esp_deep_sleep(sleepTime * 60000000);
}

void initialConfig() {
  // Power on reset, run set up and configuration
  log_i("Starting from POR...");

  // For debugging or resetting the serial number
  if (DEBUG_CLEAR_NVS) { config.clear(); };

  // Check if we have a serial number, if not, prompt for one.
  config.restore();
  if (config.serial_number == "") {
    log_e("Serial number is unset, please provide valid serial number");
    Serial.print("Enter serial number> ");
    config.serial_number = readLine().c_str();
  }

  // TODO: Could we just call web setup directly?
  //  Start web setup
  SemaphoreHandle_t web_setup_done = xSemaphoreCreateBinary();
  auto web_params = web_setup_task_params_t{};
  web_params.web_setup_done = web_setup_done;
  web_params.config = &config;
  web_params.timeout = 15 * 60; // timeout in 15 minutes
  TaskHandle_t web_setup;
  xTaskCreatePinnedToCore(web_setup_task, "web-setup", 16384, &web_params, 10,
                          &web_setup, 1);
  // Wait until web setup succeeds or times out.
  xSemaphoreTake(web_setup_done, portMAX_DELAY);
  vTaskDelete(web_setup);
  vSemaphoreDelete(web_setup_done);
  log_i("Web setup finished...");

  // Create new file for measurements
  config.logfilename = LOGFILE_PREFIX + config.serial_number + "_" +
                       timestamp().c_str() + LOGFILE_POSTFIX;
  if (!SD.exists("/data")) SD.mkdir("/data");
  log_i("Creating file %s", config.logfilename.c_str());
  auto file = SD.open(config.logfilename.c_str(), FILE_WRITE);
  if (!file) {
    log_e("SD Card failed to open!");
    for (;;) {}; // If we don't have a file we stop
  }
  file.println(config.dumps().c_str()); // Embed configuration
  file.println(header.c_str());
  file.close();

  // Save configuration to internal storage
  config.save();
  log_i("Successfully saved configuration");
}

String selfTest() {
  std::stringstream ss{};
  ss << "Power on self test\nChamber firmware v0.2" << std::endl;
  // Set up serial port for SCD30
  Serial1.begin(19200, SERIAL_8N1, PIN_UART_RX, PIN_UART_TX);
  // Check SD
  SPI.begin(PIN_SPI_SCLK, PIN_SPI_MISO, PIN_SPI_MOSI, PIN_SD_CSN);
  bool sd_ok = SD.begin(PIN_SD_CSN, SPI);
  SD.end();
  ss << "SD Card: " << (sd_ok ? "OK" : "FAIL") << std::endl;
  // Check SCD30
  auto mb = Modbus(&Serial1);
  SCD30_MB scd30;
  scd30 = SCD30_MB(&mb);
  bool scd_ok = scd30.sensor_connected();
  ss << "SCD30 CO2: " << (scd_ok ? "OK" : "FAIL") << std::endl;
  // Check BME280
  bool wire_ok = Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  ss << "I2C Bus: " << (wire_ok ? "OK" : "FAIL") << std::endl;
  Adafruit_BME280 bme280{};
  bool bme_ok = bme280.begin(0x76, &Wire);
  ss << "BME280 Temp/Hum/Pres: " << (bme_ok ? "OK" : "FAIL") << std::endl;
  // Check Temperature sensor
  OneWire oneWire(PIN_TEMP_SENSOR);
  bool onewire_ok = oneWire.reset();
  ss << "OneWire bus: " << (onewire_ok ? "OK" : "FAIL") << std::endl;
  DallasTemperature soil_temp(&oneWire);
  soil_temp.begin();
  bool temp_ok = soil_temp.getDeviceCount() == 1;
  ss << "DS18 Soil temp: " << (temp_ok ? "OK" : "FAIL") << std::endl;
  // Check moisture sensor?
  // TODO: Figure out a way to do that?
  // Check valves (this is only audible/tactile, no software readback)
  board::close_valves();
  board::open_valves();
  board::close_valves();
  board::open_valves();
  ss << "Power on self test complete";
  return ss.str().c_str();
}

void setup() {
  // Prepare hardware
  board::setup_gpio();
  board::power_on();

  // Serial debug logging
  Serial.begin(115200);
  
  config.poweronselftest = selfTest();
  log_i("%s", config.poweronselftest);

  // Set up SD card
  SPI.begin(PIN_SPI_SCLK, PIN_SPI_MISO, PIN_SPI_MOSI);
  log_fail("SD initialization...", SD.begin(PIN_SD_CSN, SPI),
           !CONTINUE_WITHOUT_SD);

  if (!experimentOngoing) {
    // Power on reset, need to start config website
    initialConfig();
  } else {
    // We're waking up from deep sleep during an ongoing experiment
    // So we just need to restore the config
    log_i("Starting from deep sleep...");
    config.restore();
  }

  vTaskDelay(100 / portTICK_PERIOD_MS);
  enterWarmup();
  if (config.chamber_type == "valve") { // TODO: refactor magic string
    vTaskDelay(config.warmup_time * 1000 / portTICK_PERIOD_MS);
    enterPremix();
    vTaskDelay(config.premix_time * 1000 / portTICK_PERIOD_MS);
    enterValvesClosed();
    vTaskDelay(config.meas_time * 1000 / portTICK_PERIOD_MS);
    enterPostmix();
    vTaskDelay(config.postmix_time * 1000 / portTICK_PERIOD_MS);
  } else {
    delay(config.flow_meas_time);
  }
  enterSleep(config.sleep_duration);
}

void loop() {
  log_e("Why are we in the loop?");
  for (;;) {};
}