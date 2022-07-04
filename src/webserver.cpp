#include <webserver.h>

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

DNSServer dnsServer;
AsyncWebServer server(80);

// Redirect all requests to the ip of the ESP (necessary for whitelist rules).
class CaptiveRequestHandler : public AsyncWebHandler {
 public:
  CaptiveRequestHandler() {}
  virtual ~CaptiveRequestHandler() {}

  bool canHandle(AsyncWebServerRequest *request) {
    // Only redirect if the requested host doesn't match
    bool res = request->host() != WiFi.softAPIP().toString();
    log_d("Host: %s", request->host().c_str());
    log_d("Redirect: %s", res ? "Yes" : "No");
    return res;
  }

  void handleRequest(AsyncWebServerRequest *request) {
    request->redirect("http://" + WiFi.softAPIP().toString() + "/");
  }
};

// Return a comma seperated list of data files on the sd card
String index_filelist(bool size = false) {
  auto root = SD.open(data_dir);
  auto ss = std::stringstream{};
  while (true) {
    auto file = root.openNextFile();
    if (!file) break;  // No more files
    log_d("File: %s", file.name());
    if (size) {
      ss << (float)file.size() << ", ";
    } else {
      ss << '"' << file.name() << "\", ";
    }
    file.close();
  }
  root.close();
  return String(ss.str().c_str());
}

// Replace %PLACEHOLDERS% in index.html with real values
String Webserver::index_template_processor(const String &var) {
  log_d("index_template called with: %s", var.c_str());
  if (var == "SERIAL_NUMBER") {
    return (*serial_number).c_str();
  } else if (var == "filelist") {
    return index_filelist();
  } else if (var == "sizelist") {
    return index_filelist(true);
  } else if (var == "co2interval") {
    return std::to_string(config.sleep_duration).c_str();
  } else if (var == "co2lograte") {
    return std::to_string(config.co2_meas_interval).c_str();
  } else if (var == "soillograte") {
    return std::to_string(config.soil_meas_interval).c_str();
  } else if (var == "warmupduration") {
    return std::to_string(config.warmup_time).c_str();
  } else if (var == "premixduration") {
    return std::to_string(config.premix_time).c_str();
  } else if (var == "valvesclosedduration") {
    return std::to_string(config.measurement_time).c_str();
  } else if (var == "postmixduration") {
    return std::to_string(config.postmix_time).c_str();
  }
  return var;
}

void web_setup_task(void* parameter) {
  // Actual meat
  WiFi.mode(WIFI_MODE_AP);
  auto softAP_ip = IPAddress{10, 0, 0, 1};
  auto gateway_ip = IPAddress{0, 0, 0, 0};
  auto subnet_mask = IPAddress{255, 255, 255, 0};
  WiFi.softAPConfig(softAP_ip, gateway_ip, subnet_mask);
  WiFi.softAP("esp-captive");

  // All roads lead to Rome
  dnsServer.start(53, "*", WiFi.softAPIP());

  // Redirect to correct url so the whitelisting works.
  server.addHandler(new CaptiveRequestHandler())
      .setFilter(ON_AP_FILTER);  // only when requested from AP

  // Index route
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *req) {
    req->send(SD, "/index.html", "text/html", false, index_template_processor);
  });

  // Delete measurement data route
  server.on("/deleteall", HTTP_GET, [](AsyncWebServerRequest *req) {
    auto dir = SD.open(data_dir);
    while (true) {
      auto file = dir.openNextFile();
      if (!file) break;
      auto path = file.path();
      file.close();
      log_d("Deleting %s,  %s", path, SD.remove(path) ? "SUCCESS" : "FAILURE");
    }
    dir.close();
    req->redirect("http://" + WiFi.softAPIP().toString() + "/");
  });

  server.on("/start", HTTP_GET, [](AsyncWebServerRequest *req) {
    config.latitude = req->getParam("lat")->value().toFloat();
    config.longitude = req->getParam("lon")->value().toFloat();
    config.sleep_duration = req->getParam("co2interval")->value().toInt();
    config.co2_meas_interval = req->getParam("co2lograte")->value().toInt();
    config.soil_meas_interval = req->getParam("soillograte")->value().toInt();
    config.warmup_time = req->getParam("warmupduration")->value().toInt();
    config.premix_time = req->getParam("premixduration")->value().toInt();
    config.measurement_time = req->getParam("valvesclosedduration")->value().toInt();
    config.postmix_time = req->getParam("postmixduration")->value().toInt();
    location_notes = std::string(req->getParam("locnotes")->value().c_str());
    setTimeFromWeb(req->getParam("timestamp")->value());
    req->send(200, "text_plain", "Submitted.");
    vTaskDelay(100 / portTICK_PERIOD_MS);
    xSemaphoreGive(web_server_status);
  });

  // Serve static files
  server.serveStatic("/", SD, "/");

  // 404 route
  server.onNotFound([](AsyncWebServerRequest *req) {
    req->send(404, "text/plain", "Not found.");
  });
  server.begin();

  while(true) {
    dnsServer.processNextRequest();
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}