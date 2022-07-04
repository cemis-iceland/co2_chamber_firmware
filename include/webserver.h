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

class Webserver {
    public:
        void web_setup_task(void* parameter);
    private:
        DNSServer dnsServer;
        AsyncWebServer server(80);

        static TaskHandle_t web_setup = NULL;
        static SemaphoreHandle_t web_server_status;

        String index_filelist(bool size = false);
        String index_template_processor(const String &var);
};