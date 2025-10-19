#include "rfm69.h"
#include "pico/stdlib.h"
#include "hardware/spi.h"

#define REG_OPMODE      0x01
#define REG_DATA        0x00
#define REG_IRQFLAGS2   0x28
#define MODE_STDBY      0x04
#define MODE_TX         0x0C
#define MODE_RX         0x10

Rfm69::Rfm69(spi_inst_t* spiPort, uint csPin, uint dio0Pin, uint resetPin)
    : spi(spiPort), cs(csPin), dio0(dio0Pin), rst(resetPin)
{
    gpio_init(cs);
    gpio_set_dir(cs, GPIO_OUT);
    gpio_put(cs, 1);

    gpio_init(dio0);
    gpio_set_dir(dio0, GPIO_IN);

    gpio_init(rst);
    gpio_set_dir(rst, GPIO_OUT);
    gpio_put(rst, 0);
}

void Rfm69::reset() {
    gpio_put(rst, 1);
    sleep_ms(10);
    gpio_put(rst, 0);
    sleep_ms(10);
}

uint8_t Rfm69::readReg(uint8_t addr) {
    uint8_t tx[2] = { (uint8_t)(addr & 0x7F), 0 };
    uint8_t rx[2] = { 0 };
    gpio_put(cs, 0);
    spi_write_read_blocking(spi, tx, rx, 2);
    gpio_put(cs, 1);
    return rx[1];
}

void Rfm69::writeReg(uint8_t addr, uint8_t value) {
    uint8_t tx[2] = { (uint8_t)(addr | 0x80), value };
    gpio_put(cs, 0);
    spi_write_blocking(spi, tx, 2);
    gpio_put(cs, 1);
}

void Rfm69::setMode(uint8_t mode) {
    uint8_t op = readReg(REG_OPMODE);
    writeReg(REG_OPMODE, (op & 0xE3) | mode);
    while (!(readReg(0x27) & 0x80));
}

void Rfm69::configureDefaults() {
    writeReg(0x01, 0x04);   // Standby
    writeReg(0x02, 0x00);   // DataModul: packet mode, FSK
    writeReg(0x03, 0x05);   // BitRateMsb  (4800 bps example)
    writeReg(0x04, 0x00);   // BitRateLsb
    writeReg(0x05, 0x52);   // FdevMsb (frequency deviation)
    writeReg(0x06, 0x83);   // FdevLsb
    writeReg(0x11, 0x9F);   // PowerLevel
    writeReg(0x25, 0x40);   // DccFreq, RxBw

    // Enable sync word (must match both ends)
    writeReg(0x2E, 0x90);   // SyncOn=1, SyncSize=2 bytes
    writeReg(0x2F, 0xAA);   // Sync word byte 1
    writeReg(0x30, 0x2D);   // Sync word byte 2

    // Variable length packets, CRC OFF
    writeReg(0x37, 0x80);   // PacketConfig1
    writeReg(0x38, 0x05);   // Payload length (ignored in var-len)
    writeReg(0x3C, 0x8F);   // FIFO threshold
    writeReg(0x3D, 0x00);   // PacketConfig2: AES off, no auto restart
}


bool Rfm69::init(float freqMHz) {
    configureDefaults();
    setMode(MODE_STDBY);
    sleep_ms(10);

    float fstep = 32e6 / 524288.0;
    uint32_t frf = (uint32_t)(freqMHz * 1e6 / fstep);

    writeReg(0x07, (frf >> 16) & 0xFF);
    writeReg(0x08, (frf >> 8) & 0xFF);
    writeReg(0x09, frf & 0xFF);

    return true;
}

bool Rfm69::send(const uint8_t* data, uint8_t len) {
    setMode(MODE_STDBY);
    while (!(readReg(0x27) & 0x80));

    writeReg(0x3D, 0x10);
    writeReg(0x00, len);
    for (uint8_t i = 0; i < len; ++i)
        writeReg(0x00, data[i]);

    setMode(MODE_TX);

    int timeout = 10000;
    while (!(readReg(REG_IRQFLAGS2) & 0x08) && --timeout) tight_loop_contents();

    setMode(MODE_STDBY);
    sleep_us(50);
    return timeout > 0;
}

bool Rfm69::receive(uint8_t* buffer, uint8_t& len) {
    uint8_t irq2 = readReg(0x28);
    if ((irq2 & 0x04) == 0) return false; // PayloadReady

    uint8_t l = readReg(0x00);
    if(l > 64) return false; // prevent overflow

    for (uint8_t i = 0; i < l; ++i)
        buffer[i] = readReg(0x00);

    len = l;
    return true;
}


