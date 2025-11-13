#include <SPI.h>
#include <RH_RF69.h>
#include <math.h>
#include <Wire.h>
#include "Adafruit_seesaw.h"
#define RF69_FREQ 915.0
#define RFM69_CS   13
#define RFM69_RST  15
#define RFM69_INT  14
#define LED        25  // Use 25 for onboard LED, or 13 if you wired an external LED
const int redPin = 0;
const int greenPin = 2;
const int bluePin = 1;
const int yellowLedPin = 3;
RH_RF69 rf69(RFM69_CS, RFM69_INT);

void setup() {
  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);
  pinMode(yellowLedPin, OUTPUT);

  Serial.begin(115200);
  //delay(2000); // Wait for USB serial to initialize
  while (!Serial) delay(1); // Wait for Serial Console (comment out line if no computer)

  analogWrite(yellowLedPin, 255);
  RFM_Setup();
}

void RFM_Setup(){
  //Serial.println("Initialized");

  pinMode(RFM69_RST, OUTPUT);
  digitalWrite(RFM69_RST, LOW);

  // manual reset
  digitalWrite(RFM69_RST, HIGH);
  delay(10);
  digitalWrite(RFM69_RST, LOW);
  delay(10);

  if (!rf69.init()) {
    Serial.println("RFM69 radio init failed");
    while (1);
  }
  //Serial.println("RFM69 radio init OK");

  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM (for low power module)
  // No encryption
  if (!rf69.setFrequency(RF69_FREQ)) {
    Serial.println("setFrequency failed");
  }
  else {
    //Serial.println("setFrequency success");
  }

  // If you are using a high power RF69 eg RFM69HW, you *must* set a Tx power with the
  // ishighpowermodule flag set like this:
  rf69.setTxPower(20, true);  // range from 14-20 for power, 2nd arg must be true for 69HCW

  // The encryption key has to be the same as the one in the server
  uint8_t key[] = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
                    0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
  rf69.setEncryptionKey(key);

  Serial.print("RFM69 radio @");  Serial.print((int)RF69_FREQ);  Serial.println(" MHz");
}

double t = 0.0;
double last_received_message = 0.0;
//bool ledIsOn = false;
void loop() {
  //if (!rf69.available()) return;
  //Serial.println(counter);
  delay(10);
  t += 0.01;

  long steps = lround(t * 100.0); // counter in 0.01 units
  if (steps % 100 == 0) {
      int32_t position = encoder.getEncoderPosition();
      Serial.print("Position: ");
      Serial.println(position);
  }

  SetLED();
  ReceiveRfmMessage();
}

void ReceiveRfmMessage(){
  uint8_t buf[RH_RF69_MAX_MESSAGE_LEN];
  uint8_t len = sizeof(buf);
  if (rf69.recv(buf, &len)) {
    if (!len) return;
    buf[len] = 0;
    Serial.print("Received [");
    Serial.print(len);
    Serial.print("]: ");
    Serial.println((char*)buf);
    Serial.print("RSSI: ");
    Serial.println(rf69.lastRssi(), DEC);

    last_received_message = t;

    if (strstr((char *)buf, "Hello World")) {
      // Send a reply!
      uint8_t data[] = "response";
      rf69.send(data, sizeof(data));
      rf69.waitPacketSent();
      Serial.println("Sent a reply");
    }
  }
}

void SetLED() {
  double timeSinceMsg = t - last_received_message;
  int brightness;
  if (timeSinceMsg < 2.0) {
    // Nonlinear fade (ease-out effect)
    double ratio = timeSinceMsg / 2.0;
    brightness = (int)(255.0 * pow(1.0 - ratio, 2.0));
  } else {
    brightness = 0;
  }
  analogWrite(greenPin, brightness);
}
