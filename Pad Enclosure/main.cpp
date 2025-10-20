#include "rfm69.h"
#include <iostream>
#include <cstring>
#include <unistd.h>
#include <thread>
#include <chrono>

// Copy to pi:
// scp -r "C:\Users\matt\Documents\GitHub\GSE-Embedded-Software\Pad Enclosure\."  matt@10.0.0.210:/home/matt/Desktop/Pad-Enclosure/
// Compile on Pi:
// cmake ..; make

int main() {
    Rfm69 radio("/dev/spidev0.0", 25); // SPI0 CE0, DIO0 on GPIO25
    radio.reset(17);                   // optional reset pin (GPIO17)
    radio.init(915.0);                 // set frequency 915 MHz
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    uint8_t version = radio.readReg(0x10);
    std::cout << "RegVersion = 0x" << std::hex << (int)version << std::endl;
    if (version != 0x24) {
        std::cerr << "RFM69 not detected\n";
        return 1;
    }

    while (true) {
        uint8_t msg[] = "Hello from Pi4 TX";
        if (radio.send(msg, sizeof(msg)))
        std::cout << "Packet sent successfully\n";
        else
            std::cout << "TX timeout\n";
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    //std::cout << "Sent messages\n";
    return 0;
}
