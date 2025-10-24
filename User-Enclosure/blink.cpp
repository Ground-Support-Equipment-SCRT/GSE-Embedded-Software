#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "rfm69.h"
#include <cstdio>
#include <cstring>

int main() {
    stdio_init_all();

    // Delay for setup
    for (int i = 7; i > 0; i--) {
        printf("%d\n", i);
        sleep_ms(1000);
    }

    Rfm69 radio(spi1, 13, 15, 14); // CS=13, RESET=15, DIO0=14
    radio.init();

    uint8_t version = radio.readReg(0x10);
    printf("RFM69 Version: 0x%02X\n", version);
    if ((int)version == 0) {
        printf("RFM69 not detected!\n");
        return -1;
    }

    sleep_ms(100);
    uint8_t opMode = radio.readReg(0x01);
    uint8_t mode = (opMode >> 2) & 0x07;
    printf("Current Mode: 0x%02X\n", mode);

    const char *message = "Hello World";
    uint8_t len = (uint8_t)std::strlen(message);

    while (true) {
        printf("Transmitting: %s\n", message);
        radio.send((const uint8_t *)message, len);
        sleep_ms(1000);
    }

    return 0;
}
