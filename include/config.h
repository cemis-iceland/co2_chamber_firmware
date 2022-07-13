#ifndef CONFIG_H_20220712
#define CONFIG_H_20220712
#include <FS.h>
#include <string>

/* Manages configuration including de/serialization to json */
// Configuration structure for an experiment
struct Config {
  float latitude = 64.136978;
  float longitude = -21.945821;
  int warmup_time = 300;      // Seconds spent warming up sensors
  int premix_time = 180;      // Seconds spent mixing air before measuring
  int measurement_time = 240; // Seconds spent with the chamber closed
  int postmix_time = 180;     // Seconds spent mixing air before measuring
  int co2_meas_interval = 2;  /* The interval at which the SCD30 takes
                                 measurements in seconds */
  int soil_meas_interval = 6; /* The interval at which the soil temp and soil
                                 moist sensors take measurements in seconds.
                                 Default 60s */
  int sleep_duration = 180;   // The sleep duration in minutes
  std::string logfilename = "temp.csv"; // File where measurements are stored.

  // NOT SERIALIZED
  std::string location_notes = "";
  std::string serial_number = "";

  /* Attempts to parse the configuration file at path on the given filesystem */
  void readFrom(FS& fs, const char* path);

  /** Write the current configuration to the given path on the given filesytem.
  Example: config.writeTo(SD, "config.json"); */
  void writeTo(FS& fs, const char* path);
};

#endif // CONFIG_H_20220712