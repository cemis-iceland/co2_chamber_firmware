#include "SCD30_MB.hpp"
#include "modbus.h"
#include <Arduino.h>
#include <array>
#include <iomanip>
#include <sstream>
#include <stdint.h>

// #define SCD30_DEBUG
#include "SCD30_MB.hpp"

inline std::string bytes_to_str(const uint8_t* buf, size_t len) {
  std::stringstream ss{""};
  for (int i = 0; i < len; i++) {
    ss << std::hex << std::setw(2) << std::setfill('0') << (int)buf[i] << " ";
  }
  return ss.str();
}

SCD30_MB::SCD30_MB(Modbus* modbus) : mb{modbus} {}

bool SCD30_MB::sensor_connected() {
  static const auto req =
      mb->create_request(ADDRESS, READ, reg::FIRMWARE_VERSION, 1);
  uint8_t resp[1]{0};
  mb->send_request(req, resp, 1);
  return resp[0] == SCD30_MB::ADDRESS; // True if sensor responds.
}

scd30_err_t SCD30_MB::block_until_data_ready(unsigned int timeout_ms,
                                             unsigned int poll_period_ms) {
  time_t t_spent = 0;
  bool ready = false;
  this->data_ready(&ready);
  while (!ready) {
    vTaskDelay(poll_period_ms / portTICK_PERIOD_MS);
    t_spent += poll_period_ms;
    if (t_spent >= timeout_ms) {
      return scd30_err_t::TIMEOUT;
    }
    this->data_ready(&ready);
  }
  return scd30_err_t::OK;
}

/// Poll the sensor until it has data ready, then read it.
/// Timeout is not very accurate.
/// Returns scd30_err_t::TIMEOUT in case of timeout.
scd30_err_t SCD30_MB::read_measurement_blocking(SCD30_Measurement* out,
                                                unsigned int timeout_ms,
                                                unsigned int poll_period_ms) {
  block_until_data_ready(timeout_ms, poll_period_ms);
  return read_measurement(out);
}

/// Ask the sensor whether it has a new measurement ready
/// @param out pointer to bool where result will be stored.
scd30_err_t SCD30_MB::data_ready(bool* out) {
  const auto request =
      mb->create_request(ADDRESS, READ, reg::DATA_READY, 0x0001);
  uint8_t response_buf[7]{0};

  mb->send_request(request, response_buf, sizeof(response_buf));

  if (response_buf[1] != 0x03) {
    log_e("Invalid response to data ready request: ");
    log_e("%s", bytes_to_str(response_buf, 7).c_str());
    return scd30_err_t::INVALID_RESPONSE;
  }
  *out = (bool)response_buf[4];
  return scd30_err_t::OK;
}

/// Ask the sensor for the latest measured values and store them in out
scd30_err_t SCD30_MB::read_measurement(SCD30_Measurement* out) {
  const auto read_meas_request =
      mb->create_request(ADDRESS, READ, reg::READ_DATA, 0x0006);
  uint8_t response_buf[17]{0};

  mb->send_request(read_meas_request, response_buf, sizeof(response_buf));

  if (response_buf[1] != 0x03) {
    log_e("Invalid response to read measurements request: ");
    log_e("%s", bytes_to_str(response_buf, 17).c_str());
    return scd30_err_t::INVALID_RESPONSE;
  }

  uint32_t co2 = __bswap_32(*(uint32_t*)(response_buf + 3));
  uint32_t temperature = __bswap_32(*(uint32_t*)(response_buf + 7));
  uint32_t humidity = __bswap_32(*(uint32_t*)(response_buf + 11));
  // The below code type puns to float and outputs the vales immediately.
  memcpy(&out->co2, &co2, sizeof(co2));
  memcpy(&out->temperature, &temperature, sizeof(temperature));
  memcpy(&out->humidity_percent, &humidity, sizeof(humidity));

  return scd30_err_t::OK;
}

/// Starts measuring co2 at a set interval, with optional pressure compensation.
/// @param pressure Atmospheric pressure in mbar, or 0 to disable compensation.
scd30_err_t SCD30_MB::start_cont_measurements(uint16_t pressure) {
  auto req = mb->create_request(ADDRESS, WRITE, reg::START_CONT_MEAS,
                                pressure);
  uint8_t response_buf[8]{0};
  mb->send_request(req, response_buf, sizeof(response_buf));
  return scd30_err_t::OK;
}

scd30_err_t SCD30_MB::set_meas_interval(uint16_t interval_s) {
  auto req = mb->create_request(ADDRESS, WRITE, reg::MEAS_INTERVAL,
                                interval_s);
  uint8_t response_buf[8]{0};
  mb->send_request(req, response_buf, sizeof(response_buf));
  return scd30_err_t::OK;
}