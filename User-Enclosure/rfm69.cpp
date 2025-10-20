#include "rfm69.h"
#include "pico/stdlib.h"

#define REG_OPMODE        0x01
#define REG_FIFO          0x00
#define REG_IRQFLAGS2     0x28
#define REG_RSSIVALUE     0x24
#define REG_DATAMODUL     0x02
#define REG_BITRATEMSB    0x03
#define REG_BITRATELSB    0x04
#define REG_FDEVMSB       0x05
#define REG_FDEVLSB       0x06
#define REG_FRFMSB        0x07
#define REG_FRFMID        0x08
#define REG_FRFLSB        0x09
#define REG_PACKETCONFIG1 0x37
#define REG_PAYLOADLEN    0x38
#define REG_FIFOTHRESH    0x3C
#define REG_OPMODE_RX     0x10
#define REG_OPMODE_STDBY  0x04

Rfm69::Rfm69(spi_inst_t *spiPort, uint csPin, uint resetPin, uint dio0Pin)
    : spi(spiPort), cs(csPin), rst(resetPin), dio0(dio0Pin) {}

void Rfm69::csSelect()   { gpio_put(cs, 0); }
void Rfm69::csDeselect() { gpio_put(cs, 1); }

void Rfm69::reset() {
    gpio_put(rst, 1);
    sleep_ms(100);
    gpio_put(rst, 0);
    sleep_ms(100);
}

void Rfm69::init() {
    gpio_init(cs);
    gpio_set_dir(cs, GPIO_OUT);
    csDeselect();

    gpio_init(rst);
    gpio_set_dir(rst, GPIO_OUT);
    reset();

    gpio_init(dio0);
    gpio_set_dir(dio0, GPIO_IN);

    spi_init(spi, 1000000);
    gpio_set_function(11, GPIO_FUNC_SPI); // MOSI
    gpio_set_function(12, GPIO_FUNC_SPI); // MISO
    gpio_set_function(10, GPIO_FUNC_SPI); // SCK

    // Standby
    writeReg(REG_OPMODE, REG_OPMODE_STDBY);
    sleep_ms(10);

    // Basic config for FSK packet mode, 915MHz example
    writeReg(REG_DATAMODUL, 0x00);      // packet mode, FSK
    writeReg(REG_BITRATEMSB, 0x1A);     // 4.8 kbps
    writeReg(REG_BITRATELSB, 0x0B);
    writeReg(REG_FDEVMSB, 0x00);        // 5kHz deviation
    writeReg(REG_FDEVLSB, 0x52);
    writeReg(REG_FRFMSB, 0xE4);         // 915 MHz
    writeReg(REG_FRFMID, 0xC0);
    writeReg(REG_FRFLSB, 0x00);
    writeReg(REG_PACKETCONFIG1, 0x90);  // variable length, CRC on
    writeReg(REG_PAYLOADLEN, 66);
    writeReg(REG_FIFOTHRESH, 0x8F);     // TxStart on FIFO not empty, 15-byte threshold

    setMode(REG_OPMODE_RX);
}

uint8_t Rfm69::readReg(uint8_t addr) {
    uint8_t tx[2] = { addr & 0x7F, 0 };
    uint8_t rx[2];
    csSelect();
    spi_write_read_blocking(spi, tx, rx, 2);
    csDeselect();
    return rx[1];
}

void Rfm69::writeReg(uint8_t addr, uint8_t value) {
    uint8_t buf[2] = { addr | 0x80, value };
    csSelect();
    spi_write_blocking(spi, buf, 2);
    csDeselect();
}

void Rfm69::setMode(uint8_t mode) {
    uint8_t val = readReg(REG_OPMODE) & 0xE3;
    writeReg(REG_OPMODE, val | mode);
}

bool Rfm69::receive(uint8_t *buf, uint8_t &len) {
    if (!(readReg(REG_IRQFLAGS2) & 0x04)) return false; // PayloadReady

    csSelect();
    uint8_t addr = REG_FIFO & 0x7F;
    spi_write_blocking(spi, &addr, 1);
    spi_read_blocking(spi, 0, &len, 1);  // first byte is length
    spi_read_blocking(spi, 0, buf, len);
    csDeselect();

    return true;
}
