#pragma once
// Copyright (c) 2026 Richard Vidal-Dorsch
// SPDX-License-Identifier: MIT OR Apache-2.0

#include "II2CBus.hpp"
#include "ISPIBus.hpp"
#include <cstdint>
#include <cstring>

class BME280 {
public:
    struct CalibrationData {
        uint16_t dig_T1; int16_t dig_T2, dig_T3;
        uint16_t dig_P1; int16_t dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;
    };

    BME280(II2CBus& bus, uint8_t addr = 0x76)
        : m_i2c(&bus), m_spi(nullptr), m_addr(addr), m_type(Type::I2C) {}

    BME280(ISPIBus& spi)
        : m_i2c(nullptr), m_spi(&spi), m_addr(0), m_type(Type::SPI) {}

    bool begin() {
        if (!readCalibrationData()) return false;
        // Set oversampling: Temperature x1, Pressure x1, Mode: Normal
        if (!busWrite(0xF4, 0x27)) return false;
        return true;
    }

    void readSensor(float& temperature, float& pressure) {
        uint8_t data[6];
        busRead(0xF7, data, 6); // Burst read Pressure (3 bytes) and Temp (3 bytes)

        int32_t adc_P = (data[0] << 12) | (data[1] << 4) | (data[2] >> 4);
        int32_t adc_T = (data[3] << 12) | (data[4] << 4) | (data[5] >> 4);

        temperature = compensateTemperature(adc_T);
        pressure = compensatePressure(adc_P) / 100.0f; // Convert Pa to hPa
    }

private:
    enum class Type { I2C, SPI };

    II2CBus* m_i2c;
    ISPIBus* m_spi;
    uint8_t m_addr;
    Type m_type;
    CalibrationData m_calib;
    int32_t m_tFine; // Used for pressure compensation [cite: 532]

    bool readCalibrationData() {
        uint8_t buf[24];
        if (!busRead(0x88, buf, 24)) return false;
        
        m_calib.dig_T1 = (buf[1] << 8) | buf[0];
        m_calib.dig_T2 = (buf[3] << 8) | buf[2];
        m_calib.dig_T3 = (buf[5] << 8) | buf[4];
        m_calib.dig_P1 = (buf[7] << 8) | buf[6];
        m_calib.dig_P2 = (buf[9] << 8) | buf[8];
        m_calib.dig_P3 = (buf[11] << 8) | buf[10];
        m_calib.dig_P4 = (buf[13] << 8) | buf[12];
        m_calib.dig_P5 = (buf[15] << 8) | buf[14];
        m_calib.dig_P6 = (buf[17] << 8) | buf[16];
        m_calib.dig_P7 = (buf[19] << 8) | buf[18];
        m_calib.dig_P8 = (buf[21] << 8) | buf[20];
        m_calib.dig_P9 = (buf[23] << 8) | buf[22];
        return true;
    }

    bool busWrite(uint8_t reg, uint8_t value) {
        if (m_type == Type::I2C) {
            return m_i2c->write(m_addr, reg, value);
        } else {
            uint8_t tx[2] = { (uint8_t)(reg & 0x7F), value };
            uint8_t rx[2];
            return m_spi->transfer(tx, rx, 2);
        }
    }

    bool busRead(uint8_t reg, uint8_t* buffer, uint16_t length) {
        if (m_type == Type::I2C) {
            return m_i2c->read(m_addr, reg, buffer, length);
        } else {
            // SPI read: first byte is register with MSB=1, followed by dummy bytes
            if (length + 1 > 256) return false; // avoid overruns
            uint8_t tx[256];
            uint8_t rx[256];
            tx[0] = (uint8_t)(reg | 0x80);
            for (uint16_t i = 1; i < (uint16_t)(length + 1); ++i) tx[i] = 0x00;
            if (!m_spi->transfer(tx, rx, (uint32_t)(length + 1))) return false;
            std::memcpy(buffer, rx + 1, length);
            return true;
        }
    }

    // Fixed-point compensation formulas from datasheet [cite: 541, 552]
    float compensateTemperature(int32_t adc_T) {
        int32_t var1, var2;
        var1 = ((((adc_T >> 3) - ((int32_t)m_calib.dig_T1 << 1))) * ((int32_t)m_calib.dig_T2)) >> 11;
        var2 = (((((adc_T >> 4) - ((int32_t)m_calib.dig_T1)) * ((adc_T >> 4) - ((int32_t)m_calib.dig_T1))) >> 12) * ((int32_t)m_calib.dig_T3)) >> 14;
        m_tFine = var1 + var2;
        return (float)((m_tFine * 5 + 128) >> 8) / 100.0f;
    }

    float compensatePressure(int32_t adc_P) {
        int64_t var1, var2, p;
        var1 = ((int64_t)m_tFine) - 128000;
        var2 = var1 * var1 * (int64_t)m_calib.dig_P6;
        var2 = var2 + ((var1 * (int64_t)m_calib.dig_P5) << 17);
        var2 = var2 + (((int64_t)m_calib.dig_P4) << 35);
        var1 = ((var1 * var1 * (int64_t)m_calib.dig_P3) >> 8) + ((var1 * (int64_t)m_calib.dig_P2) << 12);
        var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)m_calib.dig_P1) >> 33;
        if (var1 == 0) return 0;
        p = 1048576 - adc_P;
        p = (((p << 31) - var2) * 3125) / var1;
        var1 = (((int64_t)m_calib.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
        var2 = (((int64_t)m_calib.dig_P8) * p) >> 19;
        p = ((p + var1 + var2) >> 8) + (((int64_t)m_calib.dig_P7) << 4);
        return (float)p / 256.0f;
    }
};
