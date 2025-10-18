#pragma once
#include <cstdint>
#include <string>

class Rfm69 {
public:
    Rfm69(const std::string& spiDev = "/dev/spidev0.0", int dio0Gpio = 25);
    ~Rfm69();

    bool init(float freqMHz = 915.0);
    bool send(const uint8_t* data, uint8_t len);
    bool receive(uint8_t* buffer, uint8_t& len);
    uint8_t readReg(uint8_t addr);
    void reset(int resetGpio);

private:
    int spiFd;
    int gpioDio0;
    void writeReg(uint8_t addr, uint8_t value);
    void setMode(uint8_t mode);
    void configureDefaults();
};
