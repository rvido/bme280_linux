// Copyright (c) 2026 Richard Vidal-Dorsch
// SPDX-License-Identifier: MIT OR Apache-2.0

#include "LinuxSPIBus.hpp"
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <stdexcept>
#include <cstring>

LinuxSPIBus::LinuxSPIBus(const char* device, uint32_t speed_hz) {
    m_fd = open(device, O_RDWR);
    if (m_fd < 0) {
        throw std::runtime_error("Failed to open SPI device");
    }

    uint8_t mode = SPI_MODE_0; // BME280 supports Mode 0 and Mode 3 
    if (ioctl(m_fd, SPI_IOC_WR_MODE, &mode) < 0) {
        close(m_fd);
        throw std::runtime_error("Failed to set SPI mode");
    }
    if (ioctl(m_fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed_hz) < 0) {
        close(m_fd);
        throw std::runtime_error("Failed to set SPI max speed");
    }
    m_speed = speed_hz;
}

LinuxSPIBus::~LinuxSPIBus() {
    if (m_fd >= 0) {
        close(m_fd);
    }
}

bool LinuxSPIBus::transfer(const uint8_t* txBuffer, uint8_t* rxBuffer, uint32_t length) {
    if (m_fd < 0) return false;

    struct spi_ioc_transfer tr;
    memset(&tr, 0, sizeof(tr));
    tr.tx_buf = reinterpret_cast<uintptr_t>(txBuffer);
    tr.rx_buf = reinterpret_cast<uintptr_t>(rxBuffer);
    tr.len = length;
    tr.speed_hz = m_speed;
    tr.delay_usecs = 0;
    tr.bits_per_word = 8;
    tr.cs_change = 0;
    tr.tx_nbits = 0;
    tr.rx_nbits = 0;
    tr.word_delay_usecs = 0;
    tr.pad = 0;

    return ioctl(m_fd, SPI_IOC_MESSAGE(1), &tr) >= 0;
}
