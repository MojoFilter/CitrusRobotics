#include <Arduino.h>
#include <BleGamepad.h>
#include <map>

#include "buffbee.h"
#include <TJpg_Decoder.h>
#include "SPI.h"
#include <TFT_eSPI.h>

BleGamepad bleGamepad("BuffBee Control Panel", "Citrus Robotics Club", 100);

TFT_eSPI tft = TFT_eSPI();

std::map<uint8_t, uint8_t> pinButtons{
  { 21, BUTTON_1 },
  { 17, BUTTON_2},
  { 15, BUTTON_3}
};

std::map<uint8_t, int> buttonStates{
  { BUTTON_1, HIGH },
  { BUTTON_2, HIGH },
  { BUTTON_3, HIGH }
};

void setup() {
  Serial.begin(115200);
  
  setupButtonPins();
  bleGamepad.begin();

  tft.begin();
  tft.fillScreen(TFT_BLACK);
  TJpgDec.setJpgScale(1);
  TJpgDec.setSwapBytes(true);
  TJpgDec.setCallback(tft_output);

  drawLogo();
}

void loop() {
  if (bleGamepad.isConnected()) {
    updateButtons();
  }
}

void setupButtonPins() {
  for (auto const& [pin, button] : pinButtons) {
    pinMode(pin, INPUT_PULLUP);
  }
}

void drawLogo() {
  uint16_t w = 0, h = 0;
  TJpgDec.getJpgSize(&w, &h, buffbee, sizeof(buffbee));
  TJpgDec.drawJpg(0, 0, buffbee, sizeof(buffbee));
}

bool tft_output(int16_t x, int16_t y, uint16_t w, uint16_t h, uint16_t* bitmap)
{
   // Stop further decoding as image is running off bottom of screen
  if ( y >= tft.height() ) return 0;

  // This function will clip the image block rendering automatically at the TFT boundaries
  tft.pushImage(x, y, w, h, bitmap);

  // This might work instead if you adapt the sketch to use the Adafruit_GFX library
  // tft.drawRGBBitmap(x, y, bitmap, w, h);

  // Return 1 to decode next block
  return 1;
}

void updateButtons() {
  for (auto const& [pin, button] : pinButtons) {
    int currentButtonState = digitalRead(pin);
    int previousButtonState = buttonStates[button];
    if (currentButtonState != previousButtonState) {
      if (currentButtonState == LOW) {
        bleGamepad.press(button);
      } else {
        bleGamepad.release(button);
      }
      buttonStates[button] = currentButtonState;
    }
  }
}
