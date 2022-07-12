#include "config.h"
#include "util.h"
#include <json11.hpp>

void Config::readFrom(FS& fs, const char* path) {
  if (!fs.exists(path)) {
    log_i("File %s doesn't exist, NOT creating default file", path);
    // writeConfig(); //TODO: Should we write a default config?
    return;
  }

  auto file = fs.open(path, FILE_READ);
  if (!file) {
    // If we don't have file we stop
    log_e("SD Card failed to open!");
    for (;;) // TODO: Fault tolerance
      ;
  }
  std::string buf = "";
  while (file.available()) {
    buf += file.read();
  }
  file.close();
  std::string err;
  json11::Json configuration = json11::Json::parse(buf, err);
  // TODO: Check for failed JSON parse
  int version = configuration["version"].int_value();
  if (version == 1) {
    this->latitude = configuration["latitude"].int_value();
    this->longitude = configuration["longitude"].int_value();
    this->warmup_time = configuration["warmup_time"].int_value();
    this->premix_time = configuration["premix_time"].int_value();
    this->measurement_time = configuration["measurement_time"].int_value();
    this->postmix_time = configuration["postmix_time"].int_value();
    this->co2_meas_interval = configuration["co2_meas_interval"].int_value();
    this->soil_meas_interval = configuration["soil_meas_interval"].int_value();
    this->sleep_duration = configuration["sleep_duration"].int_value();
    this->logfilename = configuration["log_file_name"].string_value();
  } else {
    log_e("Read configuration file of unsupported version");
  }
}

void Config::writeTo(FS& fs, const char* path) {
  log_i("Creating file %s", path);
  fs.remove(path);
  auto file = fs.open(path, FILE_WRITE);
  if (!file) {
    // If we don't have file we stop //TODO: fault tolerance
    log_fail("SD Card failed to open!", false, true);
  }
  // Write config to file
  json11::Json configuration =
      json11::Json::object{{"version", (int)1},
                           {"latitude", this->latitude},
                           {"longitude", this->longitude},
                           {"warmup_time", this->warmup_time},
                           {"premix_time", this->premix_time},
                           {"measurement_time", this->measurement_time},
                           {"postmix_time", this->postmix_time},
                           {"co2_meas_interval", this->co2_meas_interval},
                           {"soil_meas_interval", this->soil_meas_interval},
                           {"sleep_duration", this->sleep_duration},
                           {"log_file_name", this->logfilename}};
  std::string conf_str = configuration.dump();
  file.write((const uint8_t*)conf_str.data(), conf_str.length());
  file.close();
}