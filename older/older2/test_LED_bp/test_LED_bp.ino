/*************************************************** 
  This is a library for our I2C LED Backpacks

  Designed specifically to work with the Adafruit LED Matrix backpacks 
  ----> http://www.adafruit.com/products/872
  ----> http://www.adafruit.com/products/871
  ----> http://www.adafruit.com/products/870

  These displays use I2C to communicate, 2 pins are required to 
  interface. There are multiple selectable I2C addresses. For backpacks
  with 2 Address Select pins: 0x70, 0x71, 0x72 or 0x73. For backpacks
  with 3 Address Select pins: 0x70 thru 0x77

  Adafruit invests time and resources providing this open source code, 
  please support Adafruit and open-source hardware by purchasing 
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.  
  BSD license, all text above must be included in any redistribution
 ****************************************************/

#include <Wire.h>
#include "Adafruit_LEDBackpack.h"
#include "Adafruit_GFX.h"

Adafruit_BicolorMatrix matrix = Adafruit_BicolorMatrix();

void setup() {
  Serial.begin(9600);
  Serial.println("8x8 LED Matrix Test");
  
  matrix.begin(0x70);  // pass in the address

  matrix.setRotation(3);
  matrix.blinkRate(0);
}

static const uint8_t PROGMEM
  left_bmp[] =
  { B10000000,
    B10000000,
    B10000000,
    B10000000,
    B10000000,
    B10000000,
    B10000000,
    B10000000 },
  right_bmp[] =
  { B00000001,
    B00000001,
    B00000001,
    B00000001,
    B00000001,
    B00000001,
    B00000001,
    B00000001 },  
  one_bmp[] =
  { B00000000,
    B00011000,
    B00111000,
    B00011000,
    B00011000,
    B00011000,
    B00111100,
    B00000000 },
   two_bmp[] =
  { B00000000,
    B00011110,
    B00110110,
    B01101100,
    B00011000,
    B00111110,
    B01111110,
    B00000000 },
    three_bmp[] =
  { B00000000,
    B00111100,
    B01100110,
    B00011100,
    B00001110,
    B01100110,
    B00111100,
    B00000000 },
    four_bmp[] =
  { B00000000,
    B00001110,
    B00011110,
    B00110110,
    B01111110,
    B00000110,
    B00000110,
    B00000000 },
  saving_data_bmp[] =
  { B00000000,
    B00000000,
    B00000000,
    B00000000,
    B00000000,
    B00000000,
    B00000000,
    B01111110 };

void left_turn() {
  matrix.drawBitmap(0, 0, left_bmp, 8, 8, LED_RED);
  matrix.writeDisplay();
}
void right_turn() {
  matrix.drawBitmap(0, 0, right_bmp, 8, 8, LED_RED);
  matrix.writeDisplay();
}
void one() {
  matrix.clear();
  matrix.drawBitmap(0, 0, one_bmp, 8, 8, LED_GREEN);
  matrix.writeDisplay();
  matrix.blinkRate(0);
}
void two() {
  matrix.clear();
  matrix.drawBitmap(0, 0, two_bmp, 8, 8, LED_RED);
  matrix.writeDisplay();
  matrix.blinkRate(0);
}
void three() {
  matrix.clear();
  matrix.drawBitmap(0, 0, three_bmp, 8, 8, LED_YELLOW);
  matrix.writeDisplay();
  matrix.blinkRate(0);
}
void four() {
  matrix.clear();
  matrix.drawBitmap(0, 0, four_bmp, 8, 8, LED_RED);
  matrix.writeDisplay();
  matrix.blinkRate(1);

}
void save_data() {
  matrix.drawBitmap(0, 0, saving_data_bmp, 8, 8, LED_YELLOW);
  matrix.writeDisplay();
  matrix.blinkRate(1);
}



void loop() {

  left_turn();
  delay(500);
  

  one();
  delay(500);
  one();
  left_turn();
  delay(500);

  two();
  delay(500);
  two();
  right_turn();
  delay(500);

  three();
  delay(500);
  three();
  left_turn();
  delay(500);

  four();
  delay(500);
  four();

  
  save_data();
  delay(500);
  four();
  delay(500);

  matrix.clear();
  delay(1000);


  
  /*
  matrix.clear();
  matrix.drawBitmap(0, 0, one_bmp, 8, 8, LED_RED);
  matrix.writeDisplay();
  delay(500);

  
  
  matrix.clear();
  delay(500);
  
  matrix.clear();
  matrix.drawBitmap(0, 0, two_bmp, 8, 8, LED_RED);
  matrix.writeDisplay();
  delay(500);

   
  matrix.clear();
  delay(500);
  
  matrix.clear();
  matrix.drawBitmap(0, 0, three_bmp, 8, 8, LED_RED);
  matrix.writeDisplay();
  delay(1000);

  matrix.clear();
  matrix.drawBitmap(0, 0, saving_data_bmp, 8, 8, LED_YELLOW);
  matrix.writeDisplay();
  delay(500);

  //matrix.clear();
  matrix.drawBitmap(0, 0, left_bmp, 8, 8, LED_RED);
  matrix.writeDisplay();
  delay(500);
  */
  matrix.setTextWrap(false);  // we dont want text to wrap so it scrolls nicely
  matrix.setTextSize(1);
  matrix.setTextColor(LED_GREEN);

  int num_bag_files = 60; 
  String stringOne = " bag files";
  String stringThree = num_bag_files + stringOne;
  Serial.println(stringThree);
  matrix.setTextColor(LED_GREEN);

// write python encoders and C++ decoders

// -32767 to 32767

// bagfile count
// -00xxx bagfiles, 0 to 999

// working and not working arduinos (green vs red)
// -0x000 not working arduino (1 to 9)

// wifi, different domains and individual ips
// -10xxx = 192.168.1.xxx
// -20xxx = 172.20.10.xxx
// -30xxx = 10.0.0.xxx
// -300xx = 10.0.0.xx
// -30000
// or wifi not connected

// GPS3a fix: -11000
// GPS3b fix: -12000
// GPS3a+GPS3b fix: -13000


  for (int8_t x=7; x>=-36; x--) {
    matrix.clear();
    matrix.setCursor(x,0);
    matrix.print(stringThree);
    matrix.writeDisplay();
    delay(100);

  String s5 = 'MSE'
  matrix.setTextColor(LED_GREEN);
  for (int8_t x=7; x>=-36; x--) {
    matrix.clear();
    matrix.setCursor(x,0);
    matrix.print(s5+" not working!");
    matrix.writeDisplay();
    delay(100);
  }
}

