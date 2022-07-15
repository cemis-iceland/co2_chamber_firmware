#include "SCD30_MB.hpp"
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
 * - Read contents from or create config file on SD
 * - Serve webpage
 * - Write input data from webpage into config file
 * - Start measurement cycle
 * - Check battery voltage and sleep indefinitely if voltage too low
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
OneWire oneWire(PIN_TEMP_SENSOR);
DallasTemperature soil_temp(&oneWire);

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
  experimentOngoing = true;
  digitalWrite(PIN_PWR_EN, LOW);
  // esp_sleep_pd_config(ESP_PD_DOMAIN_XTAL, ESP_PD_OPTION_OFF); // Turn off the
  // crystal oscillator
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

  Serial.begin(115200);

  // Set up SD card
  SPI.begin(PIN_SPI_SCLK, PIN_SPI_MISO, PIN_SPI_MOSI);
  log_fail("SD initialization...", SD.begin(PIN_SD_CSN, SPI),
           !CONTINUE_WITHOUT_SD);

  if (!experimentOngoing) {
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
    if (!file)
      log_fail("SD Card failed to open!", false,
               true); // If we don't have a file we stop
    file.println(header.c_str());
    file.print((delim + "\"Notes:").c_str());
    file.print(config.location_notes.c_str());
    file.println("\"");
    file.close();

    // Save configuration to internal storage
    config.save();
    log_i("Successfully saved configuration");
  } else {
    // We're waking up from deep sleep during an ongoing experiment
    // So we just need to restore the config
    log_i("Starting from deep sleep...");
    config.restore();
  }

  // Set up DS18B20 temperature sensor
  soil_temp.begin();

  vTaskDelay(100 / portTICK_PERIOD_MS);

  enterWarmup();
  vTaskDelay(config.warmup_time * 1000 / portTICK_PERIOD_MS);
  enterPremix();
  vTaskDelay(config.premix_time * 1000 / portTICK_PERIOD_MS);
  enterValvesClosed();
  vTaskDelay(config.meas_time * 1000 / portTICK_PERIOD_MS);
  enterPostmix();
  vTaskDelay(config.postmix_time * 1000 / portTICK_PERIOD_MS);
  enterSleep(config.sleep_duration);
}

void loop() {
  log_e("Why are we in the loop?");
  for (;;)
    ;
}