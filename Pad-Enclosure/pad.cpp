#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "rfm69.h"
#include <cstdio>
#include <cstring>

// 3V3 to VIN
// GND to GND
// G0 to GP14
// SCK to GP10
// MISO to GP12
// MOSI to GP11
// CS to GP13
// GP10 to MOSI

int main() {
    stdio_init_all();

    Rfm69 radio(spi1, 13, 15, 14); // CS=13, RESET=15, DIO0=14
    radio.init();

    // Delay for setup
    for (int i = 7; i > 0; i--) {
        printf("%d\n", i);
        sleep_ms(1000);
    }

    
    uint8_t version = 0;
    while (version == 0) {
        version = radio.readReg(0x10);
        printf("RFM69 Version: 0x%02X\n", version);
        if (version == 0) {
            printf("RFM69 not detected, retrying...\n");
            sleep_ms(1000);
        }
    }
    printf("RFM69 detected. Version: 0x%02X\n", version);


    sleep_ms(100);
    uint8_t opMode = radio.readReg(0x01);
    uint8_t mode = (opMode >> 2) & 0x07;
    printf("Current Mode: 0x%02X\n", mode);

    // Buffer for received data
    uint8_t buf[66];
    uint8_t len = 0;

    printf("Listening for packets...\n");

    while (true) {
        if (radio.receive(buf, len)) {
            printf("Received %d bytes: ", len);
            for (uint8_t i = 0; i < len; i++) {
                printf("%c", buf[i]);  // print as ASCII text
            }
            printf("\n");
        }
        sleep_ms(50);
    }


    return 0;
}
