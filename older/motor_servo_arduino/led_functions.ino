#include "led_constants.h"

Adafruit_BicolorMatrix matrix = Adafruit_BicolorMatrix();

String led_string;

void LED_setup() {
  matrix.begin(0x70);  // pass in the address
  matrix.setTextWrap(false);  // we dont want text to wrap so it scrolls nicely
  matrix.setTextSize(1);
  matrix.setTextColor(LED_GREEN);
  matrix.begin(0x70);  // pass in the address
  matrix.setRotation(3);
  matrix.blinkRate(0);
}

void network_left() {
  matrix.clear();
  matrix.drawBitmap(0, 0, left_arrow_bmp, 8, 8, LED_GREEN);
  matrix.writeDisplay();
  matrix.blinkRate(0);
}

void network_straight() {
  matrix.clear();
  matrix.drawBitmap(0, 0, straight_arrow_bmp, 8, 8, LED_GREEN);
  matrix.writeDisplay();
  matrix.blinkRate(0);
}

void network_right() {
  matrix.clear();
  matrix.drawBitmap(0, 0, right_arrow_bmp, 8, 8, LED_GREEN);
  matrix.writeDisplay();
  matrix.blinkRate(0);
}

void human_left() {
  matrix.clear();
  matrix.drawBitmap(0, 0, left_arrow_bmp, 8, 8, LED_RED);
  matrix.writeDisplay();
  matrix.blinkRate(0);
}

void human_straight() {
  matrix.clear();
  matrix.drawBitmap(0, 0, straight_arrow_bmp, 8, 8, LED_RED);
  matrix.writeDisplay();
  matrix.blinkRate(0);
}

void human_right() {
  matrix.clear();
  matrix.drawBitmap(0, 0, right_arrow_bmp, 8, 8, LED_RED);
  matrix.writeDisplay();
  matrix.blinkRate(0);
}

void network() {
  matrix.clear();
  matrix.drawBitmap(0, 0, network_bmp, 8, 8, LED_GREEN);
  matrix.writeDisplay();
  matrix.blinkRate(0);
}

void no_data() {
  matrix.clear();
  matrix.drawBitmap(0, 0, not_data_bmp, 8, 8, LED_RED);
  matrix.writeDisplay();
  matrix.blinkRate(1);
}

void calibrate() {
  matrix.clear();
  matrix.drawBitmap(0, 0, calibrate_bmp, 8, 8, LED_YELLOW);
  matrix.writeDisplay();
  matrix.blinkRate(1);
}

void led_serial_string_write() { 
  matrix.clear();
  matrix.blinkRate(0);
  while(Serial.available()) {
    led_string = Serial.readString();// read the incoming data as string
    if (led_string == "stop") {
      break;
    }
    while (Serial.available()==0) {
      Serial.println(led_string);
      Serial.println(-1.0*(led_string.length()*10));
      for (int8_t x=7; x>=  -1.0*(led_string.length()*7)   ; x--) {
        matrix.clear();
        matrix.setCursor(x,0);
        matrix.print(led_string);
        matrix.writeDisplay();
        delay(65);
        if (Serial.available()) {
          break;
        }
      }
    }
  }
}

