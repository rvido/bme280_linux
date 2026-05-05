// Copyright (c) 2026 Richard Vidal-Dorsch
// SPDX-License-Identifier: MIT OR Apache-2.0

#include "II2CBus.hpp"
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>

class LinuxI2CBus : public II2CBus {
public:
    LinuxI2CBus(const char* device) {
        m_fd = open(device, O_RDWR);
    }
    ~LinuxI2CBus() { if (m_fd >= 0) close(m_fd); }

    bool write(uint8_t devAddr, uint8_t regAddr, uint8_t value) override {
        ioctl(m_fd, I2C_SLAVE, devAddr);
        uint8_t buf[2] = {regAddr, value};
        return ::write(m_fd, buf, 2) == 2;
    }

    bool read(uint8_t devAddr, uint8_t regAddr, uint8_t* buffer, uint16_t length) override {
        ioctl(m_fd, I2C_SLAVE, devAddr);
        if (::write(m_fd, &regAddr, 1) != 1) return false;
        return ::read(m_fd, buffer, length) == length;
    }

private:
    int m_fd;
};
