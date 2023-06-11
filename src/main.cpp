#include "Sunrise_MB.h"
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
RTC_DATA_ATTR int intermix_done_count = 0;

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
// Appends the given string to the end of the current measurement file.
void write_to_measurement_file(std::string data){
  xSemaphoreTake(SD_mutex, portMAX_DELAY);
    auto file = SD.open(config.logfilename.c_str(), FILE_APPEND);
    if (!file) {
      log_e("Failed to open file %s", config.logfilename.c_str());
      digitalWrite(PIN_STATUS_LED, HIGH);
    } else {
      file.print(data.c_str());
      file.flush();
    }
    file.close();
    xSemaphoreGive(SD_mutex);
}


/** Task that measures CO2 concentration, temperature, pressure and humidity at a set interval */
void measure_co2_task(void* parameter) {
  // Set up Sunrise CO2 sensor
  Serial1.begin(19200, SERIAL_8N1, PIN_UART_RX, PIN_UART_TX);
  auto mb = Modbus(&Serial1);
  Sunrise_MB sunrise;
  sunrise = Sunrise_MB(&mb);
  sunrise.set_measurement_period(2);

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

    // Read sensor value from Sunrise and add it into ss
    float co2_measurement = 0.0;
    if(sunrise.read_measurement(&co2_measurement) != sunrise_err_t::OK) log_e("Sunrise measurement failed!");
    fmt_meas(time, ss, "co2", co2_measurement);

    // Read BME280 environmental data and add it into ss
    sensors_event_t temp_1, hume_1, pres_1;
    bme_temp->getEvent(&temp_1);
    bme_hume->getEvent(&hume_1);
    bme_pres->getEvent(&pres_1);
    fmt_meas(time, ss, "air_temperature", temp_1.temperature);
    fmt_meas(time, ss, "air_humidity", hume_1.relative_humidity);
    fmt_meas(time, ss, "air_pressure", pres_1.pressure, 9);

    // Save data to file
    write_to_measurement_file(ss.str());

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
    write_to_measurement_file(ss.str());

    vTaskDelay(config.soil_interval * 1000 / portTICK_PERIOD_MS);
  }
}

void enterWarmup() {
  log_i("Entering warmup");
  xTaskCreatePinnedToCore(measure_co2_task, "measure_co2", 16384, NULL, 10,
                          &measure_co2, 1);
  xTaskCreatePinnedToCore(measure_soil_task, "measure_soil", 16384, NULL, 10,
                          &measure_soil, 1);
}

void enterPremix() {
  log_i("Entering premix");
  board::fan_on();
  std::stringstream ss{""};
  std::string time = timestamp();
  fmt_meas(time, ss, "fan_on", 1);
  write_to_measurement_file(ss.str());
}

void enterValvesClosed() {
  log_i("Entering valves closed");
  board::close_valves();
  std::stringstream ss{""};
  std::string time = timestamp();
  fmt_meas(time, ss, "valves_closed", 1);
  write_to_measurement_file(ss.str());
}

void enterPostmix() {
  log_i("Entering postmix");
  board::open_valves();
  std::stringstream ss{""};
  std::string time = timestamp();
  fmt_meas(time, ss, "valves_closed", 0);
  write_to_measurement_file(ss.str());
}

void enterSleep(double sleepTime_minutes) {
  log_i("Entering sleep for %lld seconds", (long long)(sleepTime_minutes*60));
  if(measure_co2 != NULL){ vTaskDelete(measure_co2); }
  if(measure_soil != NULL){ vTaskDelete(measure_soil); }
  experimentOngoing = true;
  std::stringstream ss{""};
  std::string time = timestamp();
  fmt_meas(time, ss, "fan_on", 0);
  write_to_measurement_file(ss.str());
  board::power_off();
  esp_deep_sleep((uint64_t)(sleepTime_minutes * 60.0 * 1000.0) * 1000);
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
  ss << "Power on self test\nChamber firmware v0.3" << std::endl;
  // Set up serial port for Sunrise
  Serial1.begin(19200, SERIAL_8N1, PIN_UART_RX, PIN_UART_TX);
  // Check SD
  SPI.begin(PIN_SPI_SCLK, PIN_SPI_MISO, PIN_SPI_MOSI, PIN_SD_CSN);
  bool sd_ok = SD.begin(PIN_SD_CSN, SPI);
  ss << "SD Card: " << (sd_ok ? "OK" : "FAIL") << std::endl;
  // Check Sunrise
  auto mb = Modbus(&Serial1);
  Sunrise_MB sunrise;
  sunrise = Sunrise_MB(&mb);
  bool sunrise_ok = sunrise.sensor_connected();
  ss << "Sunrise CO2: " << (sunrise_ok ? "OK" : "FAIL") << std::endl;
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
  ss << "Power on self test complete";
  return ss.str().c_str();
}

void do_valve_dance(){
  // Check valves (this is only audible/tactile, no software readback)
  board::close_valves();
  board::open_valves();
  board::close_valves();
  board::open_valves();
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
  SD_mutex = xSemaphoreCreateMutex();
  SPI.begin(PIN_SPI_SCLK, PIN_SPI_MISO, PIN_SPI_MOSI);
  log_fail("SD initialization...", SD.begin(PIN_SD_CSN, SPI),
           !CONTINUE_WITHOUT_SD);

  if (!experimentOngoing) {
    // Power on reset, need to start config website
    do_valve_dance(); // Indicate that we're on
    initialConfig(); // Launches the config website and blocks until configured.
  } else {
    // We're waking up from deep sleep during an ongoing experiment
    // So we just need to restore the config
    log_i("Starting from deep sleep...");
    config.restore();
  }

  // Either begin measurement or an interstitial mixing
  if(intermix_done_count < config.intermix_times){
    // We're doing an intermix inbetween measurements
    log_d("Intermix %d of %d", intermix_done_count + 1, config.intermix_times);
    enterPremix();
    delay(config.intermix_duration * 1000);
    intermix_done_count += 1;
  } else {
    // Time to do a measurement
    enterWarmup(); // Turn sensors on
    if (config.chamber_type == "valve") { // TODO: refactor magic string
      delay(config.warmup_time * 1000);
      enterPremix(); // Turn fan on
      delay(config.premix_time * 1000);
      enterValvesClosed(); // Close valves
      delay(config.meas_time * 1000);
      enterPostmix(); // Open valves
      delay(config.postmix_time * 1000);
    } else { // Flow chamber
      delay(config.flow_meas_time * 1000);
    }
    intermix_done_count = 0;
  }
  // Deep sleep until next intermix or measurement, accounting for the time taken to measure
  enterSleep((
    (double)config.sleep_duration_minutes 
    - (config.warmup_time + config.premix_time + config.meas_time + config.postmix_time) / 60.0 
    - (config.intermix_times * config.intermix_duration) / 60.0
    ) / (config.intermix_times + 1));
}

// Unreachable, as we always enter deep sleep at the end of setup()
void loop() {
  log_e("Why are we in the loop?");
  for (;;) {};
}