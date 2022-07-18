#include "webserver.h"
#include <DNSServer.h>
#include <ESPAsyncWebServer.h>
#include <SD.h>
#include <WiFi.h>
#include <sstream>

// Redirect all requests to the ip of the ESP (necessary for whitelist rules).
class CaptiveRequestHandler : public AsyncWebHandler {
public:
  CaptiveRequestHandler() {}
  virtual ~CaptiveRequestHandler() {}

  bool canHandle(AsyncWebServerRequest* request) {
    // Only redirect if the requested host doesn't match
    bool res = request->host() != WiFi.softAPIP().toString();
    log_d("Host: %s", request->host().c_str());
    log_d("Redirect: %s", res ? "Yes" : "No");
    return res;
  }

  void handleRequest(AsyncWebServerRequest* request) {
    request->redirect("http://" + WiFi.softAPIP().toString() + "/");
  }
};

// Return a comma seperated list of data filenames or sizes on the sd card
String index_filelist(bool size = false) {
  auto root = SD.open("/data");
  auto ss = std::stringstream{};
  while (true) {
    auto file = root.openNextFile();
    if (!file) break; // No more files
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

// Replace @PLACEHOLDERS@ in index.html with real values
String index_template_processor(Config* config, const String& var) {
  log_d("index_template called with: %s", var.c_str());
  if (var == "SERIAL_NUMBER") {
    return config->serial_number.c_str();
  } else if (var == "filelist") {
    return index_filelist();
  } else if (var == "sizelist") {
    return index_filelist(true);
  } else if (var == "co2interval") {
    return std::to_string(config->sleep_duration).c_str();
  } else if (var == "co2lograte") {
    return std::to_string(config->co2_interval).c_str();
  } else if (var == "soillograte") {
    return std::to_string(config->soil_interval).c_str();
  } else if (var == "warmupduration") {
    return std::to_string(config->warmup_time).c_str();
  } else if (var == "premixduration") {
    return std::to_string(config->premix_time).c_str();
  } else if (var == "valvesclosedduration") {
    return std::to_string(config->meas_time).c_str();
  } else if (var == "postmixduration") {
    return std::to_string(config->postmix_time).c_str();
  } else if (var == "flow_meas_time") {
    return std::to_string(config->flow_meas_time).c_str();
  } else if (var == "chambertype") {
    return config->chamber_type;
  }
  return var;
}

// Set time from the timestamp submitted with the web form
void setTimeFromWeb(String time_string) {
  log_i("Time: %s", time_string.c_str());
  time_t rawtime =
      (time_t)(time_string.toDouble() / 1000); // Convert to seconds / 1000
  log_i("More time: %d", rawtime);
  struct timeval tv;
  struct timezone tz;
  tv.tv_sec = rawtime;
  tv.tv_usec = 0;
  tz.tz_minuteswest = 0;
  tz.tz_dsttime = 0;
  settimeofday(&tv, &tz);
}

// Main function
void web_setup_task(void* params) {
  web_setup_task_params_t* ps = (web_setup_task_params_t*)params;
  Config* config = ps->config;

  WiFi.mode(WIFI_MODE_AP);
  auto softAP_ip = IPAddress{10, 0, 0, 1};
  auto gateway_ip = IPAddress{0, 0, 0, 0};
  auto subnet_mask = IPAddress{255, 255, 255, 0};
  WiFi.softAPConfig(softAP_ip, gateway_ip, subnet_mask);
  WiFi.softAP("esp-captive");

  // All roads lead to Rome
  DNSServer dnsserver;
  dnsserver.start(53, "*", WiFi.softAPIP());

  // Initialize server on port 80;
  AsyncWebServer server(80);

  // Redirect to correct url so the whitelisting works.
  server.addHandler(new CaptiveRequestHandler());

  // Index route
  server.on("/", HTTP_GET, [config](AsyncWebServerRequest* req) {
    req->send(SD, "/index.html", "text/html", false,
              [config](const String& str) {
                return index_template_processor(config, str);
              });
  });
  // Delete measurement data route
  server.on("/deleteall", HTTP_GET, [](AsyncWebServerRequest* req) {
    auto dir = SD.open("/data"); // TODO: magic string, multiple uses
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

  // clang-format off
  // Configuration form submit route
  // When the form is submitted, the confugration values are passed as request 
  // parameters, accessible using req->getParam. However, getParam returns 
  // nullptr if no parameter of the given name was actually passed with the 
  // request, so we must be careful not to dereference a null pointer.
  // After setting the configuration values we then shut down the server.
  server.on("/start", HTTP_GET,
    [&server, &dnsserver, ps, config](AsyncWebServerRequest* req) {
      // Set time with as little delay as possible.
      if (req->hasParam("timestamp")) {
        setTimeFromWeb(req->getParam("timestamp")->value());
      }
      // Set rest of config's properties with a macro
      #define FROM_PARAM(out_, param_, tofunc_)                                \
        if (req->hasParam(param_)) {                                           \
        out_ = req->getParam(param_)->value() tofunc_;                         \
        }
      FROM_PARAM(config->latitude, "lat", .toFloat());
      FROM_PARAM(config->longitude, "lon", .toFloat());
      FROM_PARAM(config->sleep_duration, "co2interval", .toInt());
      FROM_PARAM(config->co2_interval, "co2lograte", .toInt());
      FROM_PARAM(config->soil_interval, "soillograte", .toInt());
      FROM_PARAM(config->warmup_time, "warmupduration", .toInt());
      FROM_PARAM(config->premix_time, "premixduration", .toInt());
      FROM_PARAM(config->meas_time, "valvesclosedduration", .toInt());
      FROM_PARAM(config->postmix_time, "postmixduration", .toInt());
      FROM_PARAM(config->location_notes, "locnotes", .c_str());
      FROM_PARAM(config->chamber_type, "chambertype", .c_str());
      FROM_PARAM(config->flow_meas_time, "flow_meas_time", .toInt());

      req->send(200, "text_plain", "Submitted.");

      // Set timeout to zero so that the server tears itself down
      ps->timeout = 0;
    });
  // clang-format on

  // Serve static files
  server.serveStatic("/", SD, "/");

  // 404 route
  server.onNotFound([](AsyncWebServerRequest* req) {
    req->send(404, "text/plain", "Not found.");
  });

  server.begin();

  // Run the server until we time out
  uint32_t start_time = millis();
  uint32_t elapsed_s = 0;
  while (elapsed_s < ps->timeout) {
    elapsed_s = (millis() - start_time) / 1000;
    dnsserver.processNextRequest();
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }

  // Timeout elapsed or form submitted, tear down server
  server.end();
  dnsserver.stop();
  WiFi.mode(WIFI_OFF);
  xSemaphoreGive(ps->web_setup_done);
  vTaskSuspend(nullptr); // Execution stops here (null suspends calling task)
  // Task that created us has responsibility of deleting us. TODO: should it?
}
