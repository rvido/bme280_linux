#pragma once
// Copyright (c) 2026 Richard Vidal-Dorsch
// SPDX-License-Identifier: MIT OR Apache-2.0

#include <cstdint>

/**
 * @brief Interface for I2C Bus communication.
 * * This abstraction allows the BME280 driver to be platform-independent.
 * It can be implemented for Linux userspace (i2c-dev), RTOS drivers, 
 * or bare-metal peripheral libraries.
 */
class II2CBus {
public:
    virtual ~II2CBus() = default;

    /**
     * @brief Writes a single byte to a specific register.
     * @param devAddr The 7-bit I2C device address.
     * @param regAddr The internal register address of the sensor.
     * @param value The 8-bit data to write.
     * @return true if successful, false otherwise.
     */
    virtual bool write(uint8_t devAddr, uint8_t regAddr, uint8_t value) = 0;

    /**
     * @brief Reads a sequence of bytes starting from a specific register.
     * @param devAddr The 7-bit I2C device address.
     * @param regAddr The starting internal register address.
     * @param buffer Pointer to the buffer where data will be stored.
     * @param length Number of bytes to read.
     * @return true if successful, false otherwise.
     */
    virtual bool read(uint8_t devAddr, uint8_t regAddr, uint8_t* buffer, uint16_t length) = 0;
};
