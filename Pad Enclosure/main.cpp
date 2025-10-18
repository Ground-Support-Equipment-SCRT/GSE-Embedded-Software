#include "rfm69.h"
#include <iostream>
#include <cstring>

int main() {
    Rfm69 radio;
    radio.reset(17);

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

    const char msg[] = "hello world";
    // Send three messages in quick succession
    for (int i = 0; i < 3; ++i) {
        bool ok = radio.send(reinterpret_cast<const uint8_t*>(msg), static_cast<uint8_t>(strlen(msg)));
        if (!ok) {
            std::cerr << "Send failed on attempt " << (i + 1) << "/3\n";
        }
        std::cout << "Sent message " << (i + 1) << std::endl;
    }

    //std::cout << "Sent 3 messages\n";
    return 0;
}
