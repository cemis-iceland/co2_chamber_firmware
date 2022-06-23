#pragma once
#include "modbus.h"
#include <Arduino.h>
#include <array>
#include <byteswap.h>
#include <stdint.h>

typedef struct SCD30_measurement {
  float co2;              // CO2 concentration in PPM
  float temperature;      // Temperature in degrees celsius
  float humidity_percent; // Relative humidity in percent
} SCD30_Measurement;

enum class scd30_err_t { OK = 0, INVALID_RESPONSE, TIMEOUT };

class SCD30_MB {
public:
  SCD30_MB(){};
  // modbus should be initialized at 19200 baud, with config SERIAL_8N1.
  SCD30_MB(Modbus* modbus);
  scd30_err_t read_measurement(SCD30_Measurement* out);
  scd30_err_t read_measurement_blocking(SCD30_Measurement* out,
                                        unsigned int timeout_ms = 3000,
                                        unsigned int poll_period_ms = 100);
  scd30_err_t start_cont_measurements(uint16_t pressure = 0x0000);
  scd30_err_t set_meas_interval(uint16_t interval_s = 2);
  scd30_err_t data_ready(bool* out);
  scd30_err_t block_until_data_ready(unsigned int timeout_ms = 10000,
                                     unsigned int poll_period_ms = 100);
  bool sensor_connected();

private:
  Modbus* mb;

  static const uint8_t READ = 0x03;    // Modbus function code
  static const uint8_t WRITE = 0x06;   // Modbus function code
  static const uint8_t ADDRESS = 0x61; // SCD30 sensor Address

  enum reg { // Register map of SCD30 sensor
    FIRMWARE_VERSION = 0x0020,
    MEAS_INTERVAL = 0x0025,   // Measurement interval in seconds [2-1800].
    DATA_READY = 0x0027,      // bool, true when data ready.
    READ_DATA = 0x0028,       // 6 words (3x 4-byte floats: co2, temp, rh).
    SOFT_RESET = 0x0034,      // write 0x0001 to soft reset sensor.
    START_CONT_MEAS = 0x0036, // write pressure value in mbar or 0x0000 to start
                              // continuous measurements with/without pres comp.
    STOP_CONT_MEAS = 0x0037,  // write 0x0001 to stop measurements.
    ALTIUDE_COMP = 0x0038,    // write altitude above sea level in meters.
                              // for built in pressure compensation.
    FORCE_CALIB = 0x0039,     // write co2 value in ppm to calibrate sens.
    AUTOCALI_ENABLE = 0x003A, // bool, write to control automatic calibration.
    TEMP_OFFSET = 0x003B,     // calibration for temp sensor in 1/100 deg c.
  };
};
