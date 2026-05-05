// Copyright (c) 2026 Richard Vidal-Dorsch
// SPDX-License-Identifier: MIT OR Apache-2.0

#include "ISPIBus.hpp"
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <stdexcept>
#include <cstring>

class LinuxSPIBus : public ISPIBus {
public:
    LinuxSPIBus(const char* device, uint32_t speed_hz = 1000000) {
        m_fd = open(device, O_RDWR);
        if (m_fd < 0) throw std::runtime_error("Failed to open SPI device");

        uint8_t mode = SPI_MODE_0; // BME280 supports Mode 0 and Mode 3 
        ioctl(m_fd, SPI_IOC_WR_MODE, &mode);
        ioctl(m_fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed_hz);
        m_speed = speed_hz;
    }

    ~LinuxSPIBus() { if (m_fd >= 0) close(m_fd); }

    bool transfer(const uint8_t* txBuffer, uint8_t* rxBuffer, uint32_t length) override {
        struct spi_ioc_transfer tr;
        memset(&tr, 0, sizeof(tr));
        tr.tx_buf = (unsigned long)txBuffer;
        tr.rx_buf = (unsigned long)rxBuffer;
        tr.len = length;
        tr.speed_hz = m_speed;
        tr.delay_usecs = 0;
        tr.bits_per_word = 8;
        tr.cs_change = 0;
        tr.tx_nbits = 0;
        tr.rx_nbits = 0;
        tr.word_delay_usecs = 0;
        tr.pad = 0;

        return ioctl(m_fd, SPI_IOC_MESSAGE(1), &tr) >= 1;
    }

private:
    int m_fd;
    uint32_t m_speed;
};
