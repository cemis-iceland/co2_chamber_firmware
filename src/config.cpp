#include "config.h"
#include "util.h"
#include <json11.hpp>

#include <Preferences.h>
#include <nvs_flash.h>

/* Returns current configuration as a JSON string */
std::string Config::dumps() {
  json11::Json configuration =
      json11::Json::object{{"version", (int)1},
                           {"latitude", this->latitude},
                           {"longitude", this->longitude},
                           {"warmup_time", this->warmup_time},
                           {"premix_time", this->premix_time},
                           {"meas_time", this->meas_time},
                           {"postmix_time", this->postmix_time},
                           {"co2_interval", this->co2_interval},
                           {"soil_interval", this->soil_interval},
                           {"sleep_duration", this->sleep_duration},
                           {"log_file_name", this->logfilename},
                           {"location_notes", this->location_notes}};
  return configuration.dump();
}

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
    this->meas_time = configuration["meas_time"].int_value();
    this->postmix_time = configuration["postmix_time"].int_value();
    this->co2_interval = configuration["co2_interval"].int_value();
    this->soil_interval = configuration["soil_interval"].int_value();
    this->sleep_duration = configuration["sleep_duration"].int_value();
    this->logfilename = configuration["log_file_name"].string_value().c_str();
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
                           {"meas_time", this->meas_time},
                           {"postmix_time", this->postmix_time},
                           {"co2_interval", this->co2_interval},
                           {"soil_interval", this->soil_interval},
                           {"sleep_duration", this->sleep_duration},
                           {"log_file_name", this->logfilename}};
  std::string conf_str = configuration.dump();
  file.write((const uint8_t*)conf_str.data(), conf_str.length());
  file.close();
}

// Save configuration to non-volatile storage using the Preferences library
void Config::save() {
  // WARNING: Max key length is 15 characters
  Preferences pref{};
  pref.begin("chamberconf"); // Make this a parameter?
  pref.putFloat("latitude", latitude);
  pref.putFloat("longitude", longitude);
  pref.putInt("warmup_time", warmup_time);
  pref.putInt("premix_time", premix_time);
  pref.putInt("meas_time", meas_time);
  pref.putInt("postmix_time", postmix_time);
  pref.putInt("co2_interval", co2_interval);
  pref.putInt("soil_interval", soil_interval);
  pref.putInt("sleep_duration", sleep_duration);
  pref.putString("logfilename", logfilename);
  pref.putString("serial_number", serial_number);
  pref.end();
}

// Restore configuration from non-volatile storage
void Config::restore() {
  // ^\W*(.+?),(.+?)\);
  // $2 = $1,$2);
  Preferences pref{};
  pref.begin("chamberconf");
  latitude = pref.getFloat("latitude", latitude);
  longitude = pref.getFloat("longitude", longitude);
  warmup_time = pref.getInt("warmup_time", warmup_time);
  premix_time = pref.getInt("premix_time", premix_time);
  meas_time = pref.getInt("meas_time", meas_time);
  postmix_time = pref.getInt("postmix_time", postmix_time);
  co2_interval = pref.getInt("co2_interval", co2_interval);
  soil_interval = pref.getInt("soil_interval", soil_interval);
  sleep_duration = pref.getInt("sleep_duration", sleep_duration);
  logfilename = pref.getString("logfilename", logfilename);
  serial_number = pref.getString("serial_number", serial_number);
  pref.end();
}

// Clear nvs (for testing)
void Config::clear() {
  nvs_flash_erase();
  nvs_flash_init();
}