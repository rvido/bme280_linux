# BME280 Platform-Agnostic C++ Driver

A high-performance, zero-allocation C++17 driver for the Bosch BME280 humidity, pressure, and temperature sensor. This project demonstrates modern embedded software practices such as hardware abstraction, dependency injection, and deterministic memory usage.

## Key Features

- **Platform independent:** Uses a pure-virtual Hardware Abstraction Layer (HAL) for I2C communication so the driver can run on anything from bare-metal MCUs to Linux userspace.
- **Production-grade math:** Implements Bosch-recommended 64-bit fixed-point compensation formulas for targets without an FPU.
- **Zero dynamic allocation:** All operations use the stack or pre-allocated objects, suitable for safety-critical and real-time systems.
- **Burst reads:** Reads sensor data in a single 6-byte burst to ensure register consistency.

## Architecture

The project follows the Dependency Inversion Principle: the `BME280` driver depends on the `II2CBus` interface rather than on concrete hardware implementations.

1. `include/II2CBus.hpp` — The abstract interface defining read/write operations.
2. `include/BME280.hpp` — Core driver logic, register mapping, and compensation math.
3. `src/LinuxI2CBus.cpp` — Linux-specific implementation using `/dev/i2c-x`.

## Sensor Specifications

- **Operating range:** −40 to +85 °C
- **Humidity:** 0 to 100% RH
- **Pressure:** 300 to 1100 hPa
- **Interfaces:** I2C (up to 3.4 MHz) and SPI (up to 10 MHz)
- **Typical current:** ~3.6 μA at 1 Hz

## Build Instructions

## SPI Notes

- **SPI Modes:** The BME280 supports SPI Mode 0 (CPOL=0, CPHA=0) and Mode 3 (CPOL=1, CPHA=1). Configure your host SPI controller accordingly before use.
- **Speed limits:** The sensor supports SPI clock frequencies up to 10 MHz; choose a safe default (e.g. 1 MHz or 4 MHz) and validate timing on your target.
- **Register addressing (SPI):** SPI transactions use the 7-bit register address; the MSB (8th bit) is reserved for the R/W flag. In other words, to read register `0xF7` you must send the address byte `0xF7 | 0x80 = 0xF7` (read) or `0x77` for a write command with the R/W bit cleared — check your platform's SPI transfer ordering.
- **3-wire vs 4-wire:** This driver assumes 4-wire SPI (separate MISO/MOSI). The BME280 also supports 3-wire SPI by enabling `spi3w_en` in the config register (`0xF5`) — enabling 3-wire requires additional host-side wiring and driver changes and is not implemented by this example.


This project uses modern CMake (target-based). To build:

```bash
mkdir -p build
cd build
cmake ..
make
```

To clean build artifacts:

```bash
make clean
```

## Usage Example

```cpp
#include "BME280.hpp"
#include "LinuxI2CBus.hpp"

int main() {
    LinuxI2CBus i2c("/dev/i2c-1");
    BME280 sensor(i2c, 0x76);

    if (sensor.begin()) {
        float temperature, pressure;
        sensor.readSensor(temperature, pressure);
    }
    return 0;
}
```

## References

- [BME280 Datasheet BST-BME280-DS002](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bme280-ds002.pdf)

