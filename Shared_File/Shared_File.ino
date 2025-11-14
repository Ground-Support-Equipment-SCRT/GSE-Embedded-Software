#include <RadioHead.h>
#include <SPI.h>
#include <RH_RF69.h>
//#include "pico/cyw43_arch.h"
#include "Adafruit_seesaw.h"
#include <Adafruit_BNO055.h>
#include <Wire.h>
#define RF69_FREQ 915.0
#define RFM69_CS   13
#define RFM69_RST  15
#define RFM69_INT  14
#define LED        25  // Use 25 for onboard LED, or 13 if you wired an external LED
const int redPin = 0;
const int greenPin = 2;
const int bluePin = 1;
RH_RF69 rf69(RFM69_CS, RFM69_INT);
Adafruit_seesaw ss;
Adafruit_BNO055 bno(55, 0x28, &Wire1);

int16_t packetnum = 0;  // packet counter, we increment per xmission
bool encoder_detected = false;
bool bno_detected = false;
bool is_tx = true;

void setup() {
  //pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  //pinMode(bluePin, OUTPUT);

  Serial.begin(115200);
  while(!Serial);
  
  RFM_Setup();
  Encoder_Setup();
  if(encoder_detected){
    is_tx = false;
  }
  Gyro_Setup();

  //SendRfmMessage();
  Serial.println("Finished setup.");
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

void Encoder_Setup(){
  Wire.setSDA(0);   // adjust if using different GPIOs
  Wire.setSCL(1);
  Wire.begin();
  
  if (ss.begin(0x36)) {
    encoder_detected = true;
  }
}

void Gyro_Setup(){
  Wire1.setSDA(6);
  Wire1.setSCL(7);
  Wire1.begin();

  if (!bno.begin()) {
      Serial.println("BNO055 not found");
  }
  else {
    bno_detected = true;
    bno.setMode(OPERATION_MODE_NDOF);   // NDOF
  }
}

double t = 0.0;
double last_received_message = 0.0;
int32_t encoder_position = 0;

void loop() {
  delay(10);
  t += 0.01;

  long steps = lround(t * 100.0); // counter in 0.01 units
  if (steps % 10 == 0) {
    //SendRfmMessage();
    //Serial.println(t);
    if(encoder_detected){
      int32_t pos = ss.getEncoderPosition();
      if(pos == 0 && encoder_position != 0){
        last_received_message = t;
        sendRfmMessage("test");
      }
      encoder_position = pos;
      Serial.println(pos);
    }

    if(bno_detected){
      auto vec = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
      Serial.print(vec.x());
      Serial.print(", ");
      Serial.print(vec.y());
      Serial.print(", ");
      Serial.println(vec.z());
    }
  }

  SetLED();
  ReceiveRfmMessage();
}

void sendRfmMessage(const char *textPayload) {
  Serial.print("Send ");
  Serial.println(textPayload);

  uint8_t packet[RH_RF69_MAX_MESSAGE_LEN];

  uint8_t payloadLen = strlen(textPayload);
  if (payloadLen + 3 > RH_RF69_MAX_MESSAGE_LEN) return;

  uint8_t index = 0;
  uint8_t startByte = 0x7E;
  uint8_t version = 1;
  uint8_t dest = 1;
  uint8_t source = 2;
  uint8_t type = 7;
  uint8_t commandID = 1;
  uint8_t sequence = 1;
  uint8_t isLastChunk = 1;

  packet[index++] = startByte;
  packet[index++] = version;
  packet[index++] = dest;
  packet[index++] = source;
  packet[index++] = type;
  packet[index++] = commandID;
  packet[index++] = sequence;
  packet[index++] = isLastChunk;
  packet[index++] = payloadLen;

  for (uint8_t i = 0; i < payloadLen; i++) {
      packet[index++] = textPayload[i];
  }

  rf69.send(packet, index);
  rf69.waitPacketSent();
}

void ReceiveRfmMessage() {
  static uint8_t buf[RH_RF69_MAX_MESSAGE_LEN];
  static char text[RH_RF69_MAX_MESSAGE_LEN];

  uint8_t len = sizeof(buf);

  if (!rf69.available()) return;
  if (!rf69.recv(buf, &len)) return;
  if (len < 9) return;

  uint8_t startByte = buf[0];
  uint8_t version = buf[1];
  uint8_t dest = buf[2];
  uint8_t source = buf[3];
  uint8_t type = buf[4];
  uint8_t commandID = buf[5];
  uint8_t sequence = buf[6];
  uint8_t isLastChunk = buf[7];
  uint8_t payloadLen = buf[8];

  if (payloadLen + 9 > len) return;

  memcpy(text, buf + 9, payloadLen);
  text[payloadLen] = '\0';

  Serial.print("Start Byte: "); Serial.println(startByte);
  Serial.print("Version: "); Serial.println(version);
  Serial.print("Dest: "); Serial.println(dest);
  Serial.print("Source: ");Serial.println(source);
  Serial.print("Type: ");Serial.println(type);
  Serial.print("Command ID: ");Serial.println(commandID);
  Serial.print("Sequence: ");Serial.println(sequence);
  Serial.print("Is Last Chunk: ");Serial.println(isLastChunk);
  Serial.print("Payload Length: ");Serial.println(payloadLen);
  Serial.print("Payload: ");Serial.println(text);

  last_received_message = t;
}


    // if (strstr((char *)buf, "Hello World")) {
    //   // Send a reply!
    //   uint8_t data[] = "response";
    //   rf69.send(data, sizeof(data));
    //   rf69.waitPacketSent();
    //   Serial.println("Sent a reply");
    // }

void SetLED() {
  double timeSinceMsg = t - last_received_message;
  int brightness;
  if (timeSinceMsg < 1.0) {
    // Nonlinear fade (ease-out effect)
    double ratio = timeSinceMsg;
    brightness = (int)(255.0 * pow(1.0 - ratio, 2.0));
  } else {
    brightness = 0;
  }
  analogWrite(greenPin, brightness);
}