#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>

#define TFT_CS   10   // MEMCS pin on the display
#define TFT_DC   9    // D/C pin
#define TFT_RST  8    // RST pin

Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC, TFT_RST);

void setup() {
  Serial.begin(115200);
  tft.begin();
  tft.fillScreen(ILI9341_BLACK);
  tft.setTextColor(ILI9341_YELLOW);
  tft.setTextSize(3);
  tft.setCursor(30, 60);
  tft.println("Display OK");
  delay(1000);
  tft.setTextColor(ILI9341_CYAN);
  tft.setTextSize(2);
  tft.setCursor(30, 100);
  tft.println("ILI9341 test");
}

void loop() {
  // simple animation to verify refresh
  for (int i = 0; i < tft.width(); i++) {
    tft.drawPixel(i, 200, ILI9341_RED);
    delay(2);
  }
}
