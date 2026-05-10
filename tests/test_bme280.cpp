#include "BME280.hpp"
#include "II2CBus.hpp"

#include <cassert>
#include <cstdint>
#include <iostream>
#include <map>
#include <vector>
#include <cmath>

class FakeI2C : public II2CBus {
public:
    // map starting register -> data bytes
    std::map<uint8_t, std::vector<uint8_t>> reads;
    std::vector<std::pair<uint8_t,uint8_t>> writes;

    bool write(uint8_t devAddr, uint8_t regAddr, uint8_t value) override {
        (void)devAddr;
        writes.emplace_back(regAddr, value);
        return true;
    }

    bool read(uint8_t devAddr, uint8_t regAddr, uint8_t* buffer, uint16_t length) override {
        (void)devAddr;
        auto it = reads.find(regAddr);
        if (it == reads.end()) {
            // If not found, zero-fill
            for (uint16_t i = 0; i < length; ++i) buffer[i] = 0;
            return true;
        }
        const auto& src = it->second;
        for (uint16_t i = 0; i < length; ++i) {
            buffer[i] = (i < src.size()) ? src[i] : 0;
        }
        return true;
    }
};

static std::vector<uint8_t> pack_u16_array(const std::vector<int32_t>& vals) {
    std::vector<uint8_t> out;
    for (auto v : vals) {
        uint16_t uv = static_cast<uint16_t>(v);
        out.push_back((uint8_t)(uv & 0xFF));
        out.push_back((uint8_t)((uv >> 8) & 0xFF));
    }
    return out;
}

int main() {
    FakeI2C bus;

    // Use realistic calibration values (from common examples)
    int32_t dig_T1 = 27504;
    int32_t dig_T2 = 26435;
    int32_t dig_T3 = -1000;
    int32_t dig_P1 = 36477;
    int32_t dig_P2 = -10685;
    int32_t dig_P3 = 3024;
    int32_t dig_P4 = 2855;
    int32_t dig_P5 = 140;
    int32_t dig_P6 = -7;
    int32_t dig_P7 = 15500;
    int32_t dig_P8 = -14600;
    int32_t dig_P9 = 6000;

    // pack into 24-byte buffer for 0x88..0xA3 region
    std::vector<int32_t> t_and_p = {
        dig_T1, dig_T2, dig_T3,
        dig_P1, dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9
    };
    std::vector<uint8_t> buf88 = pack_u16_array(t_and_p);
    // Ensure 24 bytes
    buf88.resize(24, 0);

    // humidity calibration
    uint8_t dig_H1 = 75;
    int16_t dig_H2 = 362;
    uint8_t dig_H3 = 0;
    int16_t dig_H4 = 315; // will be split
    int16_t dig_H5 = 50;  // will be split
    int8_t dig_H6 = 30;

    // hbuf packing according to driver expectations
    std::vector<uint8_t> hbuf(7);
    hbuf[0] = (uint8_t)(dig_H2 & 0xFF);
    hbuf[1] = (uint8_t)((dig_H2 >> 8) & 0xFF);
    hbuf[2] = dig_H3;
    // h4: upper 8 bits at hbuf[3], lower nibble in hbuf[4]
    hbuf[3] = (uint8_t)((dig_H4 >> 4) & 0xFF);
    hbuf[4] = (uint8_t)(((dig_H4 & 0x0F)) | ((dig_H5 & 0x0F) << 4));
    hbuf[5] = (uint8_t)((dig_H5 >> 4) & 0xFF);
    hbuf[6] = (uint8_t)dig_H6;

    // Register read mappings
    bus.reads[0x88] = buf88;
    bus.reads[0xA1] = std::vector<uint8_t>{dig_H1};
    bus.reads[0xE1] = hbuf;

    // Prepare sensor raw bytes for 0xF7 read (8 bytes: P(3), T(3), H(2))
    int32_t adc_P = 415148; // example raw pressure
    int32_t adc_T = 519888; // example raw temperature
    int32_t adc_H = 367;    // example raw humidity

    uint8_t data[8];
    data[0] = (uint8_t)((adc_P >> 12) & 0xFF);
    data[1] = (uint8_t)((adc_P >> 4) & 0xFF);
    data[2] = (uint8_t)((adc_P & 0x0F) << 4);
    data[3] = (uint8_t)((adc_T >> 12) & 0xFF);
    data[4] = (uint8_t)((adc_T >> 4) & 0xFF);
    data[5] = (uint8_t)((adc_T & 0x0F) << 4);
    data[6] = (uint8_t)((adc_H >> 8) & 0xFF);
    data[7] = (uint8_t)(adc_H & 0xFF);
    bus.reads[0xF7] = std::vector<uint8_t>(data, data+8);

    BME280 sensor(bus);

    bool ok = sensor.begin();
    assert(ok && "begin() should succeed with valid fake reads");

    // Verify configuration writes occurred
    bool wroteF2 = false, wroteF4 = false;
    for (auto &w : bus.writes) {
        if (w.first == 0xF2 && w.second == 0x01) wroteF2 = true;
        if (w.first == 0xF4 && w.second == 0x27) wroteF4 = true;
    }
    assert(wroteF2 && "Expected write to 0xF2 with 0x01");
    assert(wroteF4 && "Expected write to 0xF4 with 0x27");

    float temperature = 0.0f, pressure = 0.0f, humidity = 0.0f;
    sensor.readSensor(temperature, pressure, humidity);

    // Print raw results for debugging
    std::cout << "DBG: temperature=" << temperature << " pressure=" << pressure << " humidity=" << humidity << "\n";

    // Basic sanity checks on ranges
    assert(std::isfinite(temperature));
    assert(std::isfinite(pressure));
    assert(std::isfinite(humidity));
    assert(temperature > -40.0f && temperature < 85.0f);
    // Pressure range from sensor can vary; ensure it's finite and positive
    assert(pressure > 0.0f && pressure < 20000.0f);
    assert(humidity >= 0.0f && humidity <= 100.0f);

    std::cout << "BME280 unit test passed: T=" << temperature
              << " C, P=" << pressure << " hPa, H=" << humidity << " %\n";

    return 0;
}
