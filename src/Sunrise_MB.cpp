#include "Sunrise_MB.h"
#include "modbus.h"
#include <Arduino.h>
#include <array>
#include <iomanip>
#include <sstream>
#include <stdint.h>

inline std::string bytes_to_str(const uint8_t* buf, size_t len) {
  std::stringstream ss{""};
  for (int i = 0; i < len; i++) {
    ss << std::hex << std::setw(2) << std::setfill('0') << (int)buf[i] << " ";
  }
  return ss.str();
}

Sunrise_MB::Sunrise_MB(Modbus* modbus, uint8_t addr) : mb{modbus}, ADDRESS{addr} {}

/// Check if sensor is connected on the right address.
bool Sunrise_MB::sensor_connected() {
    log_e("sensor_connected not implemented for Sunrise!");
    return false;

    auto req = 
        mb->create_request(ADDRESS, READ_INPUT, reg::READ_CO2, 1);
    uint8_t resp[7]{0};
    mb->send_request(req, resp, sizeof(resp));
    return resp[0] == Sunrise_MB::ADDRESS;
}

/// Read sensor measurement.
/// @param out Pointer to float where result will be stored. [ppm]
sunrise_err_t Sunrise_MB::read_measurement(float* out) {
    const auto read_meas_request =
        mb->create_request(ADDRESS, READ_INPUT, reg::READ_CO2, 1);
    uint8_t response_buf[7]{0};

    mb->send_request(read_meas_request, response_buf, sizeof(response_buf));

    if (response_buf[1] != READ_INPUT) {
        log_e("Invalid response to read measurements request: ");
        log_e("%s", bytes_to_str(response_buf, 7).c_str());
        return sunrise_err_t::INVALID_RESPONSE;
    }

    uint16_t co2 = __bswap_16(*(uint16_t*)(response_buf + 3));
    
    *out = co2;

    return sunrise_err_t::OK;
}

/// Read sensor measurement.
/// @param out Pointer to float where result will be stored. [ppm]
sunrise_err_t Sunrise_MB::read_measurement_unfiltered(float* out) {
    const auto read_meas_request =
        mb->create_request(ADDRESS, READ_INPUT, reg::READ_CO2_UNFILT, 1);
    uint8_t response_buf[7]{0};

    mb->send_request(read_meas_request, response_buf, sizeof(response_buf));

    if (response_buf[1] != READ_INPUT) {
        log_e("Invalid response to read measurements request: ");
        log_e("%s", bytes_to_str(response_buf, 7).c_str());
        return sunrise_err_t::INVALID_RESPONSE;
    }

    uint16_t co2 = __bswap_16(*(uint16_t*)(response_buf + 3));
    
    *out = co2;

    return sunrise_err_t::OK;
}

// TODO:
//  Change to use the procedure descibed on page 31 in the modbus spec
//  Remove code that just makes the function return an error
/// Calibrates the sensor to 400 ppm.
sunrise_err_t Sunrise_MB::calibrate_400ppm() {
    log_e("Calibrate 400 ppm not implemented for Sunrise!");
    return sunrise_err_t::INVALID_RESPONSE;

    const auto calibrate_request =
        mb->create_request(ADDRESS, WRITE, reg::CALIBRATION, 0x7C06);
    uint8_t calibrate_buf[8]{0};
    mb->send_request(calibrate_request, calibrate_buf, sizeof(calibrate_buf));

    if (calibrate_buf[5] != 0x06) {
        log_e("Invalid response to background calibration request: ");
        log_e("%s", bytes_to_str(calibrate_buf, 8).c_str());
        return sunrise_err_t::INVALID_RESPONSE;
    }

    vTaskDelay(2500 / portTICK_PERIOD_MS); // Non blocking 2500 ms delay
    return sunrise_err_t::OK;
}

// TODO:
//  Clear calibration status register prior to calibration
/// Calibrates the sensor to 0 ppm. Manufacturer recommends using a pure nitrogen atmosphere.
sunrise_err_t Sunrise_MB::calibrate_0ppm() {
    const auto calibrate_request =
        mb->create_request_11byte(ADDRESS, WRITE, reg::CALIBRATION, 0x7C07);
    uint8_t calibrate_buf[8]{0};
    mb->send_request_11byte(calibrate_request, calibrate_buf, sizeof(calibrate_buf));

    if (calibrate_buf[5] != 0x07) {
        log_e("Invalid response to background calibration request: ");
        log_e("%s", bytes_to_str(calibrate_buf, 8).c_str());
        return sunrise_err_t::INVALID_RESPONSE;
    }

    vTaskDelay(2500 / portTICK_PERIOD_MS); // Non blocking 2500 ms delay
    return sunrise_err_t::OK;
}

// TODO:
//  Add code to check if 
/// Disables the Ambient Background Calibration feature of the sensor. Try using if getting weird long term behaviour.
sunrise_err_t Sunrise_MB::disable_ABC() {
    const auto disable_abc_request =
        mb->create_request(ADDRESS, WRITE, reg::ABC_PERIOD, 0);
    uint8_t response_buf[8]{0};

    log_e("Response check in disable_ABC funciton for Sunrise not implemented!");
    mb->send_request(disable_abc_request, response_buf, sizeof(response_buf));
    if (response_buf[1] != WRITE) {
        log_e("Invalid response to disable ABC request: ");
        log_e("%s", bytes_to_str(response_buf, 8).c_str());
        return sunrise_err_t::INVALID_RESPONSE;
    }

    return sunrise_err_t::OK;
}