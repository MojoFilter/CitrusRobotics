#include <array>
#include <map>
#include <sstream>

#include <Arduino.h>
#include <BleGamepad.h>

//#include "buffbee.h"
#include "talos.h"
#include <TJpg_Decoder.h>
#include "SPI.h"
#include <TFT_eSPI.h>
#include <SoftwareSerial.h>

#define AUDIO_TRIGGER_PIN 13
#define PRESS_DELAY 250

#define SOUNDBOARD_TX 21
#define SOUNDBOARD_RX 22

BleGamepad bleGamepad("Talos Control Panel", "Citrus Robotics Club", 100);

TFT_eSPI tft = TFT_eSPI();

std::map<uint8_t, uint8_t> pinButtons{
  { 21, BUTTON_1 },
  { 22, BUTTON_2},
  { 17, BUTTON_3},

  /* { 21, BUTTON_4 },
  { 22, BUTTON_5},
  { 17, BUTTON_6},

  { 21, BUTTON_7 },
  { 22, BUTTON_8},
  { 17, BUTTON_9},

  { 21, BUTTON_10 },
  { 22, BUTTON_11},
  { 17, BUTTON_12}, */
};

std::map<uint8_t, int> buttonStates{
  { BUTTON_1, HIGH },
  { BUTTON_2, HIGH },
  { BUTTON_3, HIGH },
  /*{ BUTTON_4, HIGH },
  { BUTTON_5, HIGH },
  { BUTTON_6, HIGH },
  { BUTTON_7, HIGH },
  { BUTTON_8, HIGH },
  { BUTTON_9, HIGH },
  { BUTTON_10, HIGH },
  { BUTTON_11, HIGH },
  { BUTTON_12, HIGH },*/
};

SoftwareSerial soundboardSerial(SOUNDBOARD_RX, SOUNDBOARD_TX);

void setup() {
  Serial.begin(115200);
  
  setupSound(AUDIO_TRIGGER_PIN);

  setupButtonPins();
  bleGamepad.begin();

  tft.begin();
  tft.fillScreen(TFT_BLACK);
  TJpgDec.setJpgScale(1);
  TJpgDec.setSwapBytes(true);
  TJpgDec.setCallback(tft_output);

  drawLogo();

  //soundboardSerial.begin()
}

void loop() {
  //if (bleGamepad.isConnected()) {
    updateButtons();
  //}
}

void setupButtonPins() {
  for (auto const& [pin, button] : pinButtons) {
    pinMode(pin, INPUT_PULLUP);
  }
}

void drawLogo() {
  uint16_t w = 0, h = 0;
  TJpgDec.getJpgSize(&w, &h, logo, sizeof(logo));
  TJpgDec.drawJpg(0, 0, logo, sizeof(logo));
}

bool tft_output(int16_t x, int16_t y, uint16_t w, uint16_t h, uint16_t* bitmap)
{
   // Stop further decoding as image is running off bottom of screen
  if ( y >= tft.height() ) return 0;

  // This function will clip the image block rendering automatically at the TFT boundaries
  tft.pushImage(x, y, w, h, bitmap);

  // Return 1 to decode next block
  return 1;
}

void updateButtons() {
  std::stringstream buttonStateDebug;
  Serial.println("Button Count: " + pinButtons.size());
  buttonStateDebug << "( ";
  for (auto const& [pin, button] : pinButtons) {
    int currentButtonState = digitalRead(pin);
    buttonStateDebug << currentButtonState << " ";
    int previousButtonState = buttonStates[button];
    if (currentButtonState != previousButtonState) {
      if (currentButtonState == LOW) {
        if (bleGamepad.isConnected()) {
          bleGamepad.press(button);
        }
        activateSound(AUDIO_TRIGGER_PIN);
      } else if (bleGamepad.isConnected()) {
          bleGamepad.release(button);
      }
      buttonStates[button] = currentButtonState;
    }
  }
  buttonStateDebug << ")";
  std::string str = buttonStateDebug.str();
  Serial.println(str.c_str());
}

void setupSound(int pin) {
  pinMode(pin, OUTPUT);
  digitalWrite(pin, HIGH);
}

void activateSound(int pin) {
  digitalWrite(pin, LOW);
  delay(PRESS_DELAY);
  digitalWrite(pin, HIGH);
}