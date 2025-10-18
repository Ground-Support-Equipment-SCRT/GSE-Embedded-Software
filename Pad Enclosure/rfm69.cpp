#include "rfm69.h"
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <unistd.h>
#include <cstring>
#include <iostream>
#include <fstream>

#define REG_OPMODE 0x01
#define REG_DATA   0x00
#define REG_IRQFLAGS2 0x28
#define MODE_STDBY 0x04
#define MODE_TX    0x0C
#define MODE_RX    0x10

Rfm69::Rfm69(const std::string& spiDev, int dio0Gpio)
    : spiFd(-1), gpioDio0(dio0Gpio)
{
    spiFd = open(spiDev.c_str(), O_RDWR);
    uint8_t mode = 0;
    ioctl(spiFd, SPI_IOC_WR_MODE, &mode);
    uint8_t bits = 8;
    ioctl(spiFd, SPI_IOC_WR_BITS_PER_WORD, &bits);
    uint32_t speed = 4000000;
    ioctl(spiFd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);

    // Export DIO0 GPIO for interrupt/poll
    std::ofstream exportG("/sys/class/gpio/export");
    exportG << gpioDio0;
    exportG.close();
}

Rfm69::~Rfm69() {
    if (spiFd >= 0) close(spiFd);
}

uint8_t Rfm69::readReg(uint8_t addr) {
    uint8_t tx[2] = { addr & 0x7F, 0 };
    uint8_t rx[2] = { 0, 0 };
    struct spi_ioc_transfer tr = { .tx_buf=(unsigned long)tx, .rx_buf=(unsigned long)rx, .len=2 };
    ioctl(spiFd, SPI_IOC_MESSAGE(1), &tr);
    return rx[1];
}

void Rfm69::writeReg(uint8_t addr, uint8_t value) {
    uint8_t tx[2] = { addr | 0x80, value };
    struct spi_ioc_transfer tr = { .tx_buf=(unsigned long)tx, .rx_buf=0, .len=2 };
    ioctl(spiFd, SPI_IOC_MESSAGE(1), &tr);
}

void Rfm69::setMode(uint8_t mode) {
    uint8_t op = readReg(REG_OPMODE);
    writeReg(REG_OPMODE, (op & 0xE3) | mode);
}

bool Rfm69::init(float freqMHz) {
    configureDefaults();
    setMode(MODE_STDBY);
    // frequency registers: Fstep = 32MHz / 2^19
    uint32_t frf = (freqMHz * 1000000.0) / (32e6 / 524288);
    writeReg(0x07, (frf >> 16) & 0xFF);
    writeReg(0x08, (frf >> 8) & 0xFF);
    writeReg(0x09, frf & 0xFF);
    return true;
}

void Rfm69::configureDefaults() {
    writeReg(0x02, 0x00); // DataModul
    writeReg(0x03, 0x05);
    writeReg(0x04, 0x00);
    writeReg(0x11, 0x9F); // PaLevel
    writeReg(0x25, 0x40); // DccFreq
}

bool Rfm69::send(const uint8_t* data, uint8_t len) {
    // 1. Standby mode
    std::cout << "Set standby mode" << std::endl;
    setMode(MODE_STDBY);
    while (!(readReg(0x27) & 0x80));

    // 2. Clear FIFO
    std::cout << "FIFO" << std::endl;
    writeReg(0x3D, 0x10); // PacketConfig2, reset FIFO
    while (!(readReg(0x27) & 0x80));

    // 3. Fill FIFO
    writeReg(0x00, len);
    for (uint8_t i = 0; i < len; ++i)
        writeReg(0x00, data[i]);

    // 4. Transmit
    setMode(MODE_TX);

    std::cout << "wait for sent flag" << std::endl;
    // 5. Wait for PacketSent flag (IRQFLAGS2 bit 3)
    uint8_t irq;
    int timeout = 10000;
    do {
        irq = readReg(0x28);
        timeout--;
    } while (!(irq & 0x08) && timeout > 0);


    std::cout << "set to standby" << std::endl;
    // 6. Return to standby
    setMode(MODE_STDBY);
    usleep(50); // ~50 µs is enough for mode transition
    return true;
}


bool Rfm69::receive(uint8_t* buffer, uint8_t& len) {
    if (!(readReg(REG_IRQFLAGS2) & 0x04)) return false; // no payload ready
    len = readReg(0x00); // FIFO
    for (uint8_t i = 0; i < len; ++i)
        buffer[i] = readReg(0x00);
    return true;
}

void Rfm69::reset(int resetGpio) {
    std::ofstream exportG("/sys/class/gpio/export");
    exportG << resetGpio;
    exportG.close();

    std::ofstream dirG("/sys/class/gpio/gpio" + std::to_string(resetGpio) + "/direction");
    dirG << "out";
    dirG.close();

    std::ofstream valG("/sys/class/gpio/gpio" + std::to_string(resetGpio) + "/value");
    valG << 1; valG.flush();
    usleep(10000); // 10 ms high pulse
    valG << 0; valG.flush();
    usleep(10000); // wait after release
}
