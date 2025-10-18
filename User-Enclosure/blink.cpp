#include "pico/stdlib.h"
#include <stdio.h>

int main() {
    stdio_init_all();
    const uint LED_PIN = 16;  // External LED on GPIO 25
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    while (true) {
        gpio_put(LED_PIN, 1);
        sleep_ms(500);
        gpio_put(LED_PIN, 0);
        sleep_ms(500);
    }
    return 0;
}