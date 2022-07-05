#include "configurator.h"
#include "logger.h"
#include <iostream>
#include <sstream>
#include <string>
#include <SD.h>
#include <json11.hpp>
#include <sys/time.h>
#include <DNSServer.h>
#ifdef ESP32
#include <AsyncTCP.h>
#include <WiFi.h>
#endif
#include "ESPAsyncWebServer.h"

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

// Set time from the time submitted with the web form
void setTimeFromWeb(String time_string) {
  log_i("Time: %s", time_string.c_str());
  time_t rawtime = (long)(strtoll(time_string.c_str(), NULL, 10) / 1000ll); // New way to convert to seconds since epoch
  //time_t rawtime = (time_t)(time_string.toDouble() / 1000); // Old way to convert to seconds since epoch
  log_i("More time: %d", rawtime);
  struct timeval tv;
  struct timezone tz;
  tv.tv_sec = rawtime;
  tv.tv_usec = 0;
  tz.tz_minuteswest = 0;
  tz.tz_dsttime = 0;
  settimeofday(&tv, &tz);
}

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
String index_filelist(bool size) {
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
String index_template_processor(const String &var) {
  log_d("index_template called with: %s", var.c_str());
  if (var == "SERIAL_NUMBER") {
    return serial_number.c_str();
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