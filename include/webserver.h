#include <Arduino.h>

#include <iostream>
#include <sstream>
#include <string>
#include <SD.h>
#include <DNSServer.h>
#ifdef ESP32
#include <AsyncTCP.h>
#include <WiFi.h>
#endif
#include "ESPAsyncWebServer.h"

static TaskHandle_t web_setup = NULL;
static SemaphoreHandle_t web_server_status;

struct Config {
  float latitude = 64.136978;
  float longitude = -21.945821;
  int warmup_time = 300; // 300
  int premix_time = 180; // 180
  int measurement_time = 240; // 240
  int postmix_time = 180; // 180
  int co2_meas_interval = 2; // The interval at which the SCD30 takes measurements in seconds
  int soil_meas_interval = 6; // The interval at which the soil temp and soil moist sensors take measurements in seconds // Default 60s
  int sleep_duration = 180; //10 800; // The sleep duration in minutes
  int chamber_type = 0; // 0 = valve, 1 = forced diffusion
  std::string logfilename = "temp.csv";
} config;

class Webserver {
    public:
        Webserver(std::string* serno) : serial_number(serno){};
        void web_setup_task(void* parameter);
    private:
        String index_filelist(bool size = false);
        String index_template_processor(const String &var);
        std::string* serial_number;
};