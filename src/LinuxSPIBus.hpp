#pragma once
// Copyright (c) 2026 Richard Vidal-Dorsch
// SPDX-License-Identifier: MIT OR Apache-2.0

#include "ISPIBus.hpp"

class LinuxSPIBus : public ISPIBus {
public:
    explicit LinuxSPIBus(const char* device, uint32_t speed_hz = 1000000);
    ~LinuxSPIBus() override;

    bool transfer(const uint8_t* txBuffer, uint8_t* rxBuffer, uint32_t length) override;

private:
    int m_fd = -1;
    uint32_t m_speed = 0;
};
