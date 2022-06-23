#include "K30_MB.h"
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

K30_MB::K30_MB(Modbus* modbus, uint8_t addr) : mb{modbus}, ADDRESS{addr} {}

/// Check if sensor is connected on the right address.
bool K30_MB::sensor_connected() {
    auto req = 
        mb->create_request(ADDRESS, READ_INPUT, reg::READ_CO2, 1);
    uint8_t resp[7]{0};
    mb->send_request(req, resp, sizeof(resp));
    return resp[0] == K30_MB::ADDRESS;
}

/// Read sensor measurement.
/// @param out Pointer to float where result will be stored. [ppm]
k30_err_t K30_MB::read_measurement(float* out) {
    const auto read_meas_request =
        mb->create_request(ADDRESS, READ_INPUT, reg::READ_CO2, 1);
    uint8_t response_buf[7]{0};

    mb->send_request(read_meas_request, response_buf, sizeof(response_buf));

    if (response_buf[1] != READ_INPUT) {
        log_e("Invalid response to read measurements request: ");
        log_e("%s", bytes_to_str(response_buf, 7).c_str());
        return k30_err_t::INVALID_RESPONSE;
    }

    uint16_t co2 = __bswap_16(*(uint16_t*)(response_buf + 3));
    
    *out = co2;

    return k30_err_t::OK;
}

/// Calibrates the sensor to 400 ppm.
k30_err_t K30_MB::calibrate_400ppm() {
    const auto clear_ack_request =
        mb->create_request(ADDRESS, WRITE, reg::ACKNOWLEDGE, 0);
    uint8_t clear_ack_buf[8]{0};
    mb->send_request(clear_ack_request, clear_ack_buf, sizeof(clear_ack_buf));
    
    if ((clear_ack_buf[2] | clear_ack_buf[3] | clear_ack_buf[4] | clear_ack_buf[5]) != 0x00) {
        log_e("Invalid response to clear acknowledgement register request: ");
        log_e("%s", bytes_to_str(clear_ack_buf, 8).c_str());
        return k30_err_t::INVALID_RESPONSE;
    }

    const auto calibrate_request =
        mb->create_request(ADDRESS, WRITE, reg::CALIBRATION, 0x7C06);
    uint8_t calibrate_buf[8]{0};
    mb->send_request(calibrate_request, calibrate_buf, sizeof(calibrate_buf));

    if (calibrate_buf[5] != 0x06) {
        log_e("Invalid response to background calibration request: ");
        log_e("%s", bytes_to_str(calibrate_buf, 8).c_str());
        return k30_err_t::INVALID_RESPONSE;
    }

    vTaskDelay(2500 / portTICK_PERIOD_MS); // Non blocking 2500 ms delay

    const auto read_ack_request =
        mb->create_request(ADDRESS, READ_HOLDING, reg::ACKNOWLEDGE, 1);
    uint8_t read_ack_buf[7]{0};
    mb->send_request(read_ack_request, read_ack_buf, sizeof(read_ack_buf));

    if ((read_ack_buf[4] & 0x20) != 0x20) {
        log_e("Calibration failed, invalid response to read acknowledgement register request: ");
        log_e("%s", bytes_to_str(read_ack_buf, 7).c_str());
        return k30_err_t::INVALID_RESPONSE;
    }

    return k30_err_t::OK;
}

/// Calibrates the sensor to 0 ppm. Manufacturer recommends using a pure nitrogen atmosphere.
k30_err_t K30_MB::calibrate_0ppm() {
    const auto clear_ack_request =
        mb->create_request(ADDRESS, WRITE, reg::ACKNOWLEDGE, 0);
    uint8_t clear_ack_buf[8]{0};
    mb->send_request(clear_ack_request, clear_ack_buf, sizeof(clear_ack_buf));
    
    if ((clear_ack_buf[2] | clear_ack_buf[3] | clear_ack_buf[4] | clear_ack_buf[5]) != 0x00) {
        log_e("Invalid response to clear acknowledgement register request: ");
        log_e("%s", bytes_to_str(clear_ack_buf, 8).c_str());
        return k30_err_t::INVALID_RESPONSE;
    }

    const auto calibrate_request =
        mb->create_request(ADDRESS, WRITE, reg::CALIBRATION, 0x7C07);
    uint8_t calibrate_buf[8]{0};
    mb->send_request(calibrate_request, calibrate_buf, sizeof(calibrate_buf));

    if (calibrate_buf[5] != 0x07) {
        log_e("Invalid response to background calibration request: ");
        log_e("%s", bytes_to_str(calibrate_buf, 8).c_str());
        return k30_err_t::INVALID_RESPONSE;
    }

    vTaskDelay(2500 / portTICK_PERIOD_MS); // Non blocking 2500 ms delay

    const auto read_ack_request =
        mb->create_request(ADDRESS, READ_HOLDING, reg::ACKNOWLEDGE, 1);
    uint8_t read_ack_buf[7]{0};
    mb->send_request(read_ack_request, read_ack_buf, sizeof(read_ack_buf));

    if ((read_ack_buf[4] & 0x40) != 0x40) {
        log_e("Calibration failed, invalid response to read acknowledgement register request: ");
        log_e("%s", bytes_to_str(read_ack_buf, 7).c_str());
        return k30_err_t::INVALID_RESPONSE;
    }

    return k30_err_t::OK;
}


/// Disables the Ambient Background Calibration feature of the sensor. Try using if getting weird long term behaviour.
k30_err_t K30_MB::disable_ABC() {
    const auto disable_abc_request =
        mb->create_request(ADDRESS, WRITE, reg::ABC_PERIOD, 0);
    uint8_t response_buf[8]{0};

    mb->send_request(disable_abc_request, response_buf, sizeof(response_buf));
    if (response_buf[1] != WRITE) {
        log_e("Invalid response to disable ABC request: ");
        log_e("%s", bytes_to_str(response_buf, 8).c_str());
        return k30_err_t::INVALID_RESPONSE;
    }

    return k30_err_t::OK;
}