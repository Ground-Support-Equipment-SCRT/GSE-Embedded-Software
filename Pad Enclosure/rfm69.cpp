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
    if (spiFd < 0) {
        std::cout << "spifd error" << std::endl;
        return;
    }
    uint8_t mode = 0;
    ioctl(spiFd, SPI_IOC_WR_MODE, &mode);
    if (ioctl(spiFd, SPI_IOC_WR_MODE, &mode) < 0) std::cout << "error 1" << std::endl;
    uint8_t bits = 8;
    ioctl(spiFd, SPI_IOC_WR_BITS_PER_WORD, &bits);
    if (ioctl(spiFd, SPI_IOC_WR_BITS_PER_WORD, &bits) < 0) std::cout << "error 2" << std::endl;
    uint32_t speed = 4000000;
    ioctl(spiFd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);
    if (ioctl(spiFd, SPI_IOC_WR_MAX_SPEED_HZ, &speed) < 0) std::cout << "error 3" << std::endl;

    std::cout << "SPI initialized" << std::endl;

    // Export DIO0 GPIO for interrupt/poll
    std::ofstream exportG("/sys/class/gpio/export");
    exportG << gpioDio0;
    exportG.close();
}

Rfm69::~Rfm69() {
    if (spiFd >= 0) close(spiFd);
}

uint8_t Rfm69::readReg(uint8_t addr) {
    if (spiFd < 0) return 0;

    uint8_t tx[2] = { (uint8_t)(addr & 0x7F), 0 };
    uint8_t rx[2] = { 0, 0 };

    struct spi_ioc_transfer tr = {};
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
    if (spiFd < 0) return;

    uint8_t tx[2] = { (uint8_t)(addr | 0x80), value };

    struct spi_ioc_transfer tr = {};
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


bool Rfm69::send(const uint8_t* data, int len) {
    // 1. Standby mode
    std::cout << "Set standby mode" << std::endl;
    setMode(MODE_STDBY);
    while (!(readReg(0x27) & 0x80));

    // 2. Clear FIFO
    std::cout << "FIFO" << std::endl;
    writeReg(0x3D, 0x10); // PacketConfig2, reset FIFO
    while (!(readReg(0x27) & 0x80));

    // 3. Fill FIFO
    std::cout << "Writing lengtha " << len << std::endl;
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
