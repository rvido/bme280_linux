#pragma once
// Copyright (c) 2026 Richard Vidal-Dorsch
// SPDX-License-Identifier: MIT OR Apache-2.0

#include <cstdint>

/**
 * @brief Interface for SPI Bus communication.
 * Standardizes SPI transactions for platform-agnostic drivers.
 */
class ISPIBus {
public:
    virtual ~ISPIBus() = default;

    /**
     * @brief Performs a full-duplex SPI transfer.
     * @param txBuffer Pointer to data to send.
     * @param rxBuffer Pointer to buffer for received data.
     * @param length Number of bytes to transfer.
     * @return true if successful.
     */
    virtual bool transfer(const uint8_t* txBuffer, uint8_t* rxBuffer, uint32_t length) = 0;
};
