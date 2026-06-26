#pragma once
// Copyright (c) 2026 Richard Vidal-Dorsch
// SPDX-License-Identifier: MIT OR Apache-2.0

#include "II2CBus.hpp"

class LinuxI2CBus : public II2CBus {
public:
    explicit LinuxI2CBus(const char* device);
    ~LinuxI2CBus() override;

    bool write(uint8_t devAddr, uint8_t regAddr, uint8_t value) override;
    bool read(uint8_t devAddr, uint8_t regAddr, uint8_t* buffer, uint16_t length) override;

private:
    int m_fd = -1;
};
