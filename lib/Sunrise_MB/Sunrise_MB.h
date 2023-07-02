// Senseair K30 library using serial (modbus)
#pragma once
#include "modbus.h"
#include <Arduino.h>
#include <array>
#include <byteswap.h>
#include <stdint.h>

enum class sunrise_err_t { OK = 0, INVALID_RESPONSE, TIMEOUT };

/* TODO:
    Change Write Single Register (0x06) to Write Multiple Registers
    Verify and update register map if needed

*/

class Sunrise_MB {
public:
  Sunrise_MB(){};
  Sunrise_MB(Modbus* modbus, uint8_t addr = 0x68);
  sunrise_err_t read_measurement(float* out);
  sunrise_err_t read_measurement_unfiltered(float* out);
  sunrise_err_t set_measurement_period(uint16_t seconds);
  sunrise_err_t disable_ABC(); // Not implemented
  sunrise_err_t calibrate_400ppm(); // Not implemented
  sunrise_err_t calibrate_0ppm();

  bool sensor_connected();
private:
  Modbus* mb;

  static const uint8_t READ_INPUT = 0x04;   // Modbus function code
  static const uint8_t READ_HOLDING = 0x03; // Modbus function code
  static const uint8_t WRITE = 0x10;        // Modbus function code
  uint8_t ADDRESS;                          // Sunrise sensor address

  enum reg {  // Register map of the Sunrise sensor
    // Input Registers
    READ_CO2 = 0x0003,
    READ_CO2_UNFILT = 0x0008,
    READ_STATUS = 0x0000,

    // Holding registers
    CALIBRATION = 0x0001,
    ABC_PERIOD = 0x000D,
    MEASUREMENT_PERIOD = 0x000B,
  };
};