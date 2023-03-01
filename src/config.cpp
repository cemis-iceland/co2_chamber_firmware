#include "config.h"
#include "util.h"
#include <json11.hpp>

#include <Preferences.h>
#include <nvs_flash.h>

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
  pref.putInt("inter_times", intermix_times);
  pref.putInt("inter_done", intermix_done_count);
  pref.putInt("inter_duration", intermix_duration);
  pref.putInt("sleep_duration", sleep_duration);
  pref.putString("logfilename", logfilename);
  pref.putString("serial_number", serial_number);
  pref.putInt("flow_meas_time", flow_meas_time);
  pref.putString("chamber_type", chamber_type);
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
  intermix_times = pref.getInt("soil_interval", intermix_times);
  intermix_done_count = pref.getInt("inter_done", intermix_done_count);
  intermix_duration = pref.getInt("inter_duration", intermix_duration);
  sleep_duration = pref.getInt("sleep_duration", sleep_duration);
  logfilename = pref.getString("logfilename", logfilename);
  serial_number = pref.getString("serial_number", serial_number);
  flow_meas_time = pref.getInt("flow_meas_time", flow_meas_time);
  chamber_type = pref.getString("chamber_type", chamber_type);
  pref.end();
}

// Clear nvs (for testing)
void Config::clear() {
  nvs_flash_erase();
  nvs_flash_init();
}

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
                           {"intermix_times", this->intermix_times},
                           {"intermix_duration", this->intermix_duration},
                           {"soil_interval", this->soil_interval},
                           {"sleep_duration", this->sleep_duration},
                           {"log_file_name", this->logfilename.c_str()},
                           {"location_notes", this->location_notes.c_str()},
                           {"flow_meas_time", this->flow_meas_time},
                           {"chamber_type", this->chamber_type.c_str()}};
  return configuration.dump();
}


