#include "K96_MB.h"
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

K96_MB::K96_MB(Modbus* modbus, uint8_t addr) : mb{modbus}, ADDRESS{addr} {}

/// Check if sensor is connected on the right address.
bool K96_MB::sensor_connected() {
    auto req = 
        mb->create_request(ADDRESS, READ_INPUT, input_reg::ErrorStatus, 1);
    uint8_t resp[7]{0};
    mb->send_request(req, resp, sizeof(resp));
    uint16_t reply = __bswap_16(*(uint16_t*)(resp + 3));
    reply = reply & 0xBBF7;
    if ((reply != 0x0000) || (resp[1] != ADDRESS)) {
        log_e("Invalid response to error status request: ");
        log_e("%s", bytes_to_str(resp, 7).c_str());
        return false;
    }
    return true;
}

/// Read sensor measurement.
/// @param co2 CO2 concentration [ppm]
/// @param h2o H2O concentration [ppm]
k96_err_t K96_MB::read_measurement(K96_Measurement* out) {
    const auto read_co2_request =
        mb->create_request(ADDRESS, READ_INPUT, input_reg::LPL_ConcPC_flt, 1);
    const auto read_h2o_request =
        mb->create_request(ADDRESS, READ_INPUT, input_reg::MPL_ConcPC_flt, 1);
    
    uint8_t co2_response_buf[7]{0};
    uint8_t h2o_response_buf[7]{0};

    vTaskDelay(5/portTICK_PERIOD_MS);
    mb->send_request(read_co2_request, co2_response_buf, sizeof(co2_response_buf));
    vTaskDelay(5/portTICK_PERIOD_MS);
    mb->send_request(read_h2o_request, h2o_response_buf, sizeof(h2o_response_buf));

    if (co2_response_buf[1] != READ_INPUT) {
        log_e("Invalid response to co2 request: ");
        log_e("%s", bytes_to_str(co2_response_buf, 7).c_str());
        return k96_err_t::INVALID_RESPONSE;
    } else if (h2o_response_buf[1] != READ_INPUT) {
        log_e("Invalid response to h2o request: ");
        log_e("%s", bytes_to_str(h2o_response_buf, 7).c_str());
        return k96_err_t::INVALID_RESPONSE;
    }

    int16_t co2meas = __bswap_16(*(uint16_t*)(co2_response_buf + 3));
    int16_t h2omeas = __bswap_16(*(uint16_t*)(h2o_response_buf + 3));
    
    (*out).co2 = (float)co2meas / 10.0;
    (*out).h2o = (float)h2omeas;

    return k96_err_t::OK;
}

/// Reads values from the built-in BME280
/// @param temp Temperature [°C]
/// @param hum Relative humidity [%]
/// @param press Pressure [hPa]
k96_err_t K96_MB::read_bme(K96_BME_Measurement* out) {
    const auto read_temp_request =
        mb->create_request(ADDRESS, READ_INPUT, input_reg::RH_T_sensor0, 1);
    const auto read_hum_request =
        mb->create_request(ADDRESS, READ_INPUT, input_reg::RH_Sensor0, 1);
    const auto read_press_request =
        mb->create_request(ADDRESS, READ_INPUT, input_reg::P_Sensor0_flt, 1);

    uint8_t temp_response[7]{0};
    uint8_t hum_response[7]{0};
    uint8_t press_response[7]{0};

    vTaskDelay(5/portTICK_PERIOD_MS);
    mb->send_request(read_temp_request, temp_response, sizeof(temp_response));
    vTaskDelay(5/portTICK_PERIOD_MS);
    mb->send_request(read_hum_request, hum_response, sizeof(hum_response));
    vTaskDelay(5/portTICK_PERIOD_MS);
    mb->send_request(read_press_request, press_response, sizeof(press_response));

    if(temp_response[1] != READ_INPUT && hum_response[1] != READ_INPUT && press_response[1] != READ_INPUT) {
        log_e("Invalid response to bme request (temp, hum, press): ");
        log_e("%s", bytes_to_str(temp_response, 7).c_str());
        log_e("%s", bytes_to_str(hum_response, 7).c_str());
        log_e("%s", bytes_to_str(press_response, 7).c_str());
        return k96_err_t::INVALID_RESPONSE;
    }

    int16_t tempMeas = __bswap_16(*(int16_t*)(temp_response + 3));
    int16_t humMeas = __bswap_16(*(int16_t*)(hum_response + 3));
    int16_t pressMeas = __bswap_16(*(int16_t*)(press_response + 3));

    (*out).temperature = tempMeas / 100.0;  // div 100.0
    (*out).humidity = humMeas / 100.0;      // div 100.0
    (*out).pressure = pressMeas / 10.0;      // mult 10.0

    return k96_err_t::OK;
}

k96_err_t K96_MB::read_iir_coeff(uint8_t* out) {
    const auto get_iir_request =
        mb->create_request_7byte(ADDRESS, READ_RAM, ram_map::Frac, 1);
    uint8_t response[6]{0};
    mb->send_request_7byte(get_iir_request, response, sizeof(response));

    log_e("%s", bytes_to_str(response, 6).c_str());

    *out = response[3];
    return k96_err_t::OK;
}

k96_err_t K96_MB::write_iir_coeff(uint8_t coeff) {
    uint16_t data = (0x01 << 8) | coeff;
    const auto write_iir_request =
        mb->create_request(ADDRESS, WRITE_RAM, ram_map::Frac, data);
    uint8_t response[4]{0};
    mb->send_request(write_iir_request, response, sizeof(response));

    if(response[1] != 0x41) {
        log_e("Invalid response to write RAM request: ");
        log_e("%s", bytes_to_str(response, 1).c_str());
        return k96_err_t::INVALID_RESPONSE;
    }

    return k96_err_t::OK;
}