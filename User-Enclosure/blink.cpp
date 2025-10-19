#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "rfm69.h"
#include <cstdio>

int main() {
    stdio_init_all();

    spi_init(spi1, 4000000);
    gpio_set_function(10, GPIO_FUNC_SPI); // SCK
    gpio_set_function(11, GPIO_FUNC_SPI); // MOSI
    gpio_set_function(12, GPIO_FUNC_SPI); // MISO

    Rfm69 radio(spi1, 13, 14, 15); // CS=13, DIO0=14, RESET=15
    radio.reset();
    radio.init(915.0); // MHz

    // gives me time to open putty
    for(int i = 7; i > 0; i--){
        printf("%d\n", i);
        sleep_ms(1000);
    }

    radio.setMode(0x10);
    printf("Radio in RX mode\n");

    uint8_t version = radio.readReg(0x10);
    printf("RFM69 Version: 0x%02X\n", version);
    if((int)version == 0){
        printf("RFM69 not detected!\n");
        return -1;
    }

    
    //const uint8_t msg[] = "Hello";
    //radio.send(msg, sizeof(msg));
    printf("Waiting for messages...\n");

    while (true) {
        uint8_t buf[64];
        uint8_t len = 0;
        if (radio.receive(buf, len)) {
            printf("Received %d bytes: ", len);
            for (int i = 0; i < len; i++) printf("%02X ", buf[i]);
            printf("\n");
        }
        sleep_ms(500);
    }
    return 0;
}