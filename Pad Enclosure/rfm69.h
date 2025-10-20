#ifndef RFM69_H
#define RFM69_H

#include <string>
#include <cstdint>

class Rfm69 {
public:
    Rfm69(const std::string& spiDev, int dio0Gpio);
    ~Rfm69();

    bool init(float freqMHz = 915.0);
    void reset(int resetGpio);

    uint8_t readReg(uint8_t addr);
    void writeReg(uint8_t addr, uint8_t value);
    void setMode(uint8_t mode);

    bool send(const uint8_t* data, int len);
    bool receive(uint8_t* buffer, uint8_t& len);

private:
    int spiFd;
    int gpioDio0;

    void configureDefaults();
};

#endif
