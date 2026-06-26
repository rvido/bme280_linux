// Copyright (c) 2026 Richard Vidal-Dorsch
// SPDX-License-Identifier: MIT OR Apache-2.0

#include "LinuxI2CBus.hpp"
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <stdexcept>

LinuxI2CBus::LinuxI2CBus(const char* device) {
    m_fd = open(device, O_RDWR);
    if (m_fd < 0) {
        throw std::runtime_error("Failed to open I2C device");
    }
}

LinuxI2CBus::~LinuxI2CBus() {
    if (m_fd >= 0) {
        close(m_fd);
    }
}

bool LinuxI2CBus::write(uint8_t devAddr, uint8_t regAddr, uint8_t value) {
    if (m_fd < 0) return false;
    if (ioctl(m_fd, I2C_SLAVE, devAddr) < 0) return false;
    uint8_t buf[2] = {regAddr, value};
    return ::write(m_fd, buf, 2) == 2;
}

bool LinuxI2CBus::read(uint8_t devAddr, uint8_t regAddr, uint8_t* buffer, uint16_t length) {
    if (m_fd < 0) return false;
    if (ioctl(m_fd, I2C_SLAVE, devAddr) < 0) return false;
    if (::write(m_fd, &regAddr, 1) != 1) return false;
    return ::read(m_fd, buffer, length) == length;
}
