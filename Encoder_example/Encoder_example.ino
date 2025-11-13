#include <Wire.h>
#include "Adafruit_seesaw.h"

Adafruit_seesaw ss;

void setup() {
    Serial.begin(115200);
    while (!Serial);

    Wire.setSDA(0);   // adjust if using different GPIOs
    Wire.setSCL(1);
    Wire.begin();

    if (!ss.begin(0x36)) {
        Serial.println("Seesaw not found!");
        while (1);
    }

    Serial.println("Seesaw encoder ready.");
    ss.getEncoderPosition();
    delay(10);
}

void loop() {
    int32_t pos = ss.getEncoderPosition();
    Serial.println(pos);
    delay(100);
}
