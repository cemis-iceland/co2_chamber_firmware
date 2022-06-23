// Senseair K30 library using serial (modbus)
#pragma once
#include "modbus.h"
#include <Arduino.h>
#include <array>
#include <byteswap.h>
#include <stdint.h>

enum class k96_err_t { OK = 0, INVALID_RESPONSE, TIMEOUT };

typedef struct K96_measurement {
  float co2;
  float h2o;
} K96_Measurement;

typedef struct K96_bme_measurement {
  float temperature;
  float humidity;
  float pressure;
} K96_BME_Measurement;

class K96_MB {
public:
  K96_MB(){};
  K96_MB(Modbus* modbus, uint8_t addr = 0x68);
  k96_err_t read_measurement(K96_Measurement* out);
  k96_err_t read_bme(K96_BME_Measurement* out);
  k96_err_t read_iir_coeff(uint8_t* out);
  k96_err_t write_iir_coeff(uint8_t coeff);

  bool sensor_connected();
private:
  Modbus* mb;

  static const uint8_t READ_HOLDING     = 0x03; // Modbus function code
  static const uint8_t READ_INPUT       = 0x04; // Modbus function code
  static const uint8_t READ_RAM         = 0x44; // Modbus function code
  static const uint8_t READ_EEPROM      = 0x46; // Modbus function code

  static const uint8_t WRITE_REGISTER   = 0x06; // Modbus function code
  static const uint8_t WRITE_RAM        = 0x41; // Modbus function code
  static const uint8_t WRITE_EEPROM     = 0x43; // Modbus function code
  
  uint8_t ADDRESS;                              // K96 sensor address

  enum input_reg {  // Input register map of the K30 sensor
    LPL_ConcPC_flt = 0x0000,
    SPL_ConcPC_flt = 0x0001,
    MPL_ConcPC_flt = 0x0002,
    P_Sensor0_flt = 0x0003,
    NTC0_Temp_flt = 0x0004,
    NTC1_Temp_flt = 0x0005,
    ADuCdie_Temp_flt = 0x0007,
    RH_Sensor0 = 0x0008,
    RH_T_sensor0 = 0x0009,
    ErrorStatus = 0x000E
  };

  enum ram_map {
    Frac = 0x01E4,
    Comm_address = 0x0020
  };
};