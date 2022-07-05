#ifndef configurator_h
#define configurator_h 1
#include <Arduino.h>

#include <iostream>
#include <sstream>
#include <string>
#include <SD.h>
#include <json11.hpp>
#include <DNSServer.h>
#ifdef ESP32
#include <AsyncTCP.h>
#include <WiFi.h>
#endif
#include "ESPAsyncWebServer.h"

#define CONF_FILE "/chamber_conf.json"

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

SemaphoreHandle_t web_server_status;

static TaskHandle_t web_setup = NULL;

DNSServer dnsServer;
AsyncWebServer server(80);

std::string location_notes;

std::string serial_number = "0-0";
const char data_dir[] = "/data";

static void web_setup_task(void* parameter);
void writeConfig();
void readConfig();
String index_filelist(bool size = false);
String index_template_processor(const String &var);
void setTimeFromWeb(String time_string);

#endif