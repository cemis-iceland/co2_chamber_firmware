#ifndef CONFIG_H_20220712
#define CONFIG_H_20220712
#include <FS.h>
#include <string>

/* Manages configuration including de/serialization to json */
// Configuration structure for an experiment
struct Config {
  float latitude = 64.136978;
  float longitude = -21.945821;
  String chamber_type = "valve"; // Type of chamber, one of [valve, flow]
  // Options for valve chambers
  int warmup_time = 300;  // Seconds spent warming up sensors
  int premix_time = 180;  // Seconds spent mixing air before measuring
  int meas_time = 240;    // Seconds spent with the chamber closed
  int postmix_time = 180; // Seconds spent mixing air before measuring
  // Options for flow chambers
  int flow_meas_time = 600; // For flow chambers, duration of each measurement.
  // Common options
  int co2_interval = 2;     /* The interval at which the SCD30 takes
                                    measurements in seconds */
  int soil_interval = 6;    /* The interval at which the soil temp and soil
                                    moist sensors take measurements in seconds.
                                    Default 60s */
  int sleep_duration = 180; // The sleep duration in minutes
  int intermix_times = 0; // Number of times to wake up and run the fan only between measurements
  int intermix_done_count = 0; // How many times we've intermixed so far, up to intermix_times
  int intermix_duration = 60; // Seconds to run fan for during intermix
  String logfilename = "temp.csv"; // File where measurements are stored.
  String serial_number = "";       // Serial number of the chamber

  // Not restored by restore();
  String location_notes = "";
  // Not restored by restore();
  String poweronselftest = ""; // TODO: this needs a better home

  /** Save the current configuration to non-volatile internal storage */
  void save();

  /** Attempt to restore configuration from non-volatile internal storage */
  void restore();

  /** (For testing) Clear saved configuration from non-volatile storage.
   * Warning: also clears everything else from non-volatile storage
   * Warning: Doesn't clear the members of the config object */
  void clear();

  /* Return the current configuration as a JSON string */
  std::string dumps();

};

#endif // CONFIG_H_20220712