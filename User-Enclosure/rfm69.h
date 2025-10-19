#ifndef RFM69_H
#define RFM69_H

#include "pico/stdlib.h"
#include "hardware/spi.h"

class Rfm69 {
public:
    Rfm69(spi_inst_t* spiPort, uint csPin, uint dio0Pin, uint resetPin);
    bool init(float freqMHz);
    bool send(const uint8_t* data, uint8_t len);
    bool receive(uint8_t* buffer, uint8_t& len);
    void reset();
    void setMode(uint8_t mode);

private:
    spi_inst_t* spi;
    uint cs;
    uint dio0;
    uint rst;

    void configureDefaults();
    uint8_t readReg(uint8_t addr);
    void writeReg(uint8_t addr, uint8_t value);
};

#endif
