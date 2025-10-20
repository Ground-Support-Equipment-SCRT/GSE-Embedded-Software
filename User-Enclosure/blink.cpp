#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "rfm69.h"
#include <cstdio>

int main() {
    stdio_init_all();


    // gives me time to open putty
    for(int i = 7; i > 0; i--){
        printf("%d\n", i);
        sleep_ms(1000);
    }

    Rfm69 radio(spi1, 13, 15, 14); // CS=13, RESET=15, DIO0=14
    radio.init();

    //radio.setMode(0x10);
    //printf("Radio in RX mode\n");

    uint8_t version = radio.readReg(0x10);
    printf("RFM69 Version: 0x%02X\n", version);
    if((int)version == 0){
        printf("RFM69 not detected!\n");
        return -1;
    }

    sleep_ms(100);
    uint8_t opMode = radio.readReg(0x01);
    uint8_t mode = (opMode >> 2) & 0x07;  // isolate bits 4–2
    printf("Current Mode: 0x%02X\n", mode);
    
    uint8_t buf[66];
    uint8_t len = 0;
    while (true) {
        if (radio.receive(buf, len)) {
            printf("Received %d bytes: ", len);
            for (uint8_t i = 0; i < len; i++) {
                printf("%02X ", buf[i]);
            }
            printf("\n");
        }
        sleep_ms(50);
    }
    return 0;
}