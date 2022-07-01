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

String index_filelist(bool size = false);
String index_template_processor(const String &var);
void web_setup_task(void* parameter);