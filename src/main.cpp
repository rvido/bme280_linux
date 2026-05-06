// Copyright (c) 2026 Richard Vidal-Dorsch
// SPDX-License-Identifier: MIT OR Apache-2.0

#include <iostream>
#include <iomanip>
#include <string>
#include "BME280.hpp"
#include "LinuxI2CBus.cpp"
#include "LinuxSPIBus.cpp"

int main(int argc, char** argv) {
    // Usage: ./app [i2c|spi]
    std::string mode = "i2c";
    if (argc > 1) mode = argv[1];

    if (mode == "spi") {
        try {
            LinuxSPIBus spi("/dev/spidev0.0");
            BME280 sensor(spi);
            if (!sensor.begin()) {
                std::cerr << "Failed to initialize BME280 (SPI)!" << std::endl;
                return 1;
            }
            while (true) {
                float temp, pres, hum;
                sensor.readSensor(temp, pres, hum);
                std::cout << "Temp: " << std::fixed << std::setprecision(2) << temp << " °C, "
                          << "Pressure: " << pres << " hPa, "
                          << "Humidity: " << hum << " %RH" << std::endl;
                sleep(1);
            }
        } catch (const std::exception& e) {
            std::cerr << "SPI init error: " << e.what() << std::endl;
            return 1;
        }
    } else {
        LinuxI2CBus i2cBus("/dev/i2c-1");
        BME280 sensor(i2cBus, 0x76); // Default I2C address is 0x76 [cite: 722]

        if (!sensor.begin()) {
            std::cerr << "Failed to initialize BME280 sensor!" << std::endl;
            return 1;
        }

        while (true) {
            float temp, pres, hum;
            sensor.readSensor(temp, pres, hum);
            std::cout << "Temp: " << std::fixed << std::setprecision(2) << temp << " °C, "
                      << "Pressure: " << pres << " hPa, "
                      << "Humidity: " << hum << " %RH" << std::endl;
            sleep(1);
        }
    }

    return 0;
}
