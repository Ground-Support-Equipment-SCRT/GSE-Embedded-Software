#ifndef RFM69_H
#define RFM69_H

#include "hardware/spi.h"
#include "hardware/gpio.h"

class Rfm69 {
public:
    Rfm69(spi_inst_t *spiPort, uint csPin, uint resetPin, uint dio0Pin);
    void init();
    void reset();
    uint8_t readReg(uint8_t addr);
    void writeReg(uint8_t addr, uint8_t value);
    void setMode(uint8_t mode);
    bool receive(uint8_t *buf, uint8_t &len);
    void send(const uint8_t *data, uint8_t len);

private:
    spi_inst_t *spi;
    uint cs;
    uint rst;
    uint dio0;

    void csSelect();
    void csDeselect();
};

#endif
