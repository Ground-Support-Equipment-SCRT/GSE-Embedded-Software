#include "rfm69.h"
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <fstream>
#include <iostream>

#define REG_OPMODE      0x01
#define REG_DATA        0x00
#define REG_IRQFLAGS2   0x28
#define REG_FRFMSB      0x07
#define REG_FRFMID      0x08
#define REG_FRFLSB      0x09
#define MODE_STDBY      0x04
#define MODE_TX         0x0C
#define MODE_RX         0x10

Rfm69::Rfm69(const std::string& spiDev, int dio0Gpio)
    : spiFd(-1), gpioDio0(dio0Gpio)
{
    spiFd = open(spiDev.c_str(), O_RDWR);
    if (spiFd < 0) {
        perror("open spidev");
        return;
    }

    uint8_t mode = 0; // SPI mode 0
    uint8_t bits = 8;
    uint32_t speed = 4000000;

    if (ioctl(spiFd, SPI_IOC_WR_MODE, &mode) < 0) perror("SPI_IOC_WR_MODE");
    if (ioctl(spiFd, SPI_IOC_WR_BITS_PER_WORD, &bits) < 0) perror("SPI_IOC_WR_BITS_PER_WORD");
    if (ioctl(spiFd, SPI_IOC_WR_MAX_SPEED_HZ, &speed) < 0) perror("SPI_IOC_WR_MAX_SPEED_HZ");

    std::ofstream exportG("/sys/class/gpio/export");
    exportG << gpioDio0;
    exportG.close();
    usleep(50000);

    std::ofstream dirG("/sys/class/gpio/gpio" + std::to_string(gpioDio0) + "/direction");
    dirG << "in";
    dirG.close();
}

Rfm69::~Rfm69() {
    if (spiFd >= 0) close(spiFd);
}

uint8_t Rfm69::readReg(uint8_t addr) {
    uint8_t tx[2] = { (uint8_t)(addr & 0x7F), 0x00 };
    uint8_t rx[2] = { 0, 0 };

    struct spi_ioc_transfer tr{};
    tr.tx_buf = (unsigned long)tx;
    tr.rx_buf = (unsigned long)rx;
    tr.len = 2;
    tr.speed_hz = 4000000;
    tr.bits_per_word = 8;

    if (ioctl(spiFd, SPI_IOC_MESSAGE(1), &tr) < 0)
        perror("SPI_IOC_MESSAGE");
    return rx[1];
}

void Rfm69::writeReg(uint8_t addr, uint8_t value) {
    uint8_t tx[2] = { (uint8_t)(addr | 0x80), value };

    struct spi_ioc_transfer tr{};
    tr.tx_buf = (unsigned long)tx;
    tr.len = 2;
    tr.speed_hz = 4000000;
    tr.bits_per_word = 8;

    if (ioctl(spiFd, SPI_IOC_MESSAGE(1), &tr) < 0)
        perror("SPI_IOC_MESSAGE");
}

void Rfm69::setMode(uint8_t mode) {
    uint8_t op = readReg(REG_OPMODE);
    writeReg(REG_OPMODE, (op & 0xE3) | mode);
}

void Rfm69::configureDefaults() {
    writeReg(0x02, 0x00); // FSK packet mode
    writeReg(0x03, 0x1A); // bitrate 4.8 kbps
    writeReg(0x04, 0x0B);
    writeReg(0x05, 0x00); // deviation 5kHz
    writeReg(0x06, 0x52);
    writeReg(0x37, 0x90); // variable length + CRC on
    writeReg(0x38, 66);
    writeReg(0x3C, 0x8F);
}

bool Rfm69::init(float freqMHz) {
    configureDefaults();
    setMode(MODE_STDBY);
    usleep(10000);

    uint32_t frf = (uint32_t)((freqMHz * 1e6) / (32e6 / 524288.0));
    writeReg(REG_FRFMSB, (frf >> 16) & 0xFF);
    writeReg(REG_FRFMID, (frf >> 8) & 0xFF);
    writeReg(REG_FRFLSB, frf & 0xFF);

    return true;
}

bool Rfm69::send(const uint8_t* data, int len) {
    setMode(MODE_STDBY);
    while (!(readReg(0x27) & 0x80)); // wait for mode ready

    writeReg(0x3D, 0x10); // reset FIFO
    writeReg(REG_DATA, len);
    for (int i = 0; i < len; i++) writeReg(REG_DATA, data[i]);

    setMode(MODE_TX);

    int timeout = 10000;
    while (!(readReg(REG_IRQFLAGS2) & 0x08) && --timeout > 0) usleep(100);

    setMode(MODE_STDBY);
    return timeout > 0;
}

bool Rfm69::receive(uint8_t* buffer, uint8_t& len) {
    if (!(readReg(REG_IRQFLAGS2) & 0x04)) return false; // payload ready
    len = readReg(REG_DATA);
    for (uint8_t i = 0; i < len; i++)
        buffer[i] = readReg(REG_DATA);
    return true;
}

void Rfm69::reset(int resetGpio) {
    std::ofstream exportG("/sys/class/gpio/export");
    exportG << resetGpio;
    exportG.close();
    usleep(10000);

    std::ofstream dirG("/sys/class/gpio/gpio" + std::to_string(resetGpio) + "/direction");
    dirG << "out";
    dirG.close();

    std::ofstream valG("/sys/class/gpio/gpio" + std::to_string(resetGpio) + "/value");
    valG << 1; valG.flush();
    usleep(10000);
    valG << 0; valG.flush();
    usleep(10000);
}
