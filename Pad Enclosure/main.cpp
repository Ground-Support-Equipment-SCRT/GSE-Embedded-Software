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
    Rfm69 radio;
    radio.init();
    radio.reset(17);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    uint8_t version = radio.readReg(0x10);
    std::cout << "RegVersion = 0x" << std::hex << (int)version << std::endl;
    if (version != 0x24) {
        std::cerr << "RFM69 not detected\n";
        return 1;
    }

    if (!radio.init(915.0)) {
        std::cerr << "Init failed\n";
        return 1;
    }
    
    std::cout << "Initialized" << std::endl;

    std::string msg = "hello world";
    std::cout << std::dec << "len: " << static_cast<int>(msg.size()) << std::endl;
    // Send three messages in quick succession
    while (true) {
        bool ok = radio.send(
            reinterpret_cast<const uint8_t*>(msg.data()),
            static_cast<int>(msg.size()));
        if (!ok) {
            std::cerr << "Send failed \n";
        }
        std::cout << "Sent message" << std::endl;
        
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    }

    //std::cout << "Sent 3 messages\n";
    return 0;
}
