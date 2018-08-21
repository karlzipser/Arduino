
///////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////

#include <Wire.h>
#include "Adafruit_LEDBackpack.h"
#include "Adafruit_GFX.h"

Adafruit_BicolorMatrix matrix = Adafruit_BicolorMatrix();

const int led_LEFT =  2;
const int led_RIGHT = 11;
const int led_DATA =  3;
const int led_STATE_1 = 8;
const int led_STATE_2 = 9;
const int led_STATE_3 = 10;
const int led_STATE_4 = 12;
/*
const int button_A =  5;
const int button_B =  6;
const int button_C =  7;
const int button_D =  4;

int button_A_state = LOW;
int button_B_state = LOW;
int button_C_state = LOW;
int button_D_state = LOW;
*/

static const uint8_t PROGMEM



  Calibrate_bmp[] =
  { B00000000,
    B00101000,
    B01101100,
    B11000110,
    B11000110,
    B11000110,
    B01111100,
    B00111000 },
    
  _bmp[] =
  { B00000000,
    B00000000,
    B00000000,
    B00000000,
    B00000000,
    B00000000,
    B00000000,
    B00000000 },

   X_bmp[] =
  { B10000001,
    B01000010,
    B00100100,
    B00011000,
    B00011000,
    B00100100,
    B01000010,
    B10000001 },

  one_bmp[] =
  { B00000000,
    B00100000,
    B01100000,
    B11111100,
    B11111100,
    B01100000,
    B00100000,
    B00000000 },
  two_bmp[] =
  { B00000000,
    B00000000,
    B00000000,
    B00011000,
    B00011000,
    B01111110,
    B00111100,
    B00011000 },
  three_bmp[] =
  { B00011000,
    B00111100,
    B01111110,
    B00011000,
    B00011000,
    B00000000,
    B00000000,
    B00000000 },


   one_follow_bmp[] =
  { B10000000,
    B10001000,
    B10011000,
    B10111111,
    B10111111,
    B10011000,
    B10001000,
    B10000000 },
  two_follow_bmp[] =
  { B00000000,
    B00011000,
    B00011000,
    B01111110,
    B00111100,
    B00011000,
    B00000000,
    B11111111
    },
  three_follow_bmp[] =
  {
    B11111111,
    B00000000,
    B00011000,
    B00111100,
    B01111110,
    B00011000,
    B00011000,
    B00000000
    },




  one_furtive_bmp[] =
  { B00000000,
    B00100000,
    B01000000,
    B11111100,
    B01000000,
    B00100000,
    B00000000,
    B00000000 },
  two_furtive_bmp[] =
  { B00000000,
    B00000000,
    B00000000,
    B00001000,
    B00001000,
    B00101010,
    B00011100,
    B00001000 },
  three_furtive_bmp[] =
  { B00001000,
    B00011100,
    B00101010,
    B00001000,
    B00001000,
    B00000000,
    B00000000,
    B00000000 },





  one_play_bmp[] =
  { B00010000,
    B00110000,
    B01111111,
    B11111111,
    B11111111,
    B01111111,
    B00110000,
    B00010000 },
  two_play_bmp[] =
  { B00111100,
    B00111100,
    B00111100,
    B00111100,
    B11111111,
    B01111110,
    B00111100,
    B00011000 },
  three_play_bmp[] =
  { B00011000,
    B00111100,
    B01111110,
    B11111111,
    B00111100,
    B00111100,
    B00111100,
    B00111100 },




  
  line_bmp[] =
  { B00000000,
    B00000000,
    B00000000,
    B00000000,
    B00000000,
    B00000000,
    B00000000,
    B01111110 };



void clear_matrix() {
  matrix.clear();
}

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(5);
  matrix.begin(0x70);  // pass in the address
  matrix.setRotation(3);
  matrix.blinkRate(0);
  
  matrix.drawBitmap(0, 0, line_bmp, 8, 8, LED_YELLOW);
  matrix.writeDisplay();
  matrix.blinkRate(1); 
}

long unsigned int start_time = millis();

void loop() {
  if (millis() - start_time < 10000) {
    Serial.println("('GPS2',0,0,0,0,0,0,0,0,0.00000,0.00000,0.00,0.00,0.00,0)"); // dummy data
    delay(100);   
  }
  unsigned int parsed_int = Serial.parseInt();
  if (parsed_int > 0) {
      //Serial.println(parsed_int);
      int parsed_mode = parsed_int/10000;
      int parsed_val_1 = (parsed_int-parsed_mode*10000)/100;
      int parsed_val_2 = (parsed_int-parsed_val_1*100-parsed_mode*10000);
      int save_mode = parsed_mode;
      int run_mode = parsed_val_2;
      int agent = parsed_val_1;
    
    int led_color = 0;
    if (agent == 1) {
      led_color = LED_RED;}
    else if (agent == 2) {
      led_color = LED_GREEN;}
    else {led_color = LED_YELLOW;}
    matrix.clear();
  
    if (run_mode == 4) {
        matrix.drawBitmap(0, 0, Calibrate_bmp, 8, 8, LED_YELLOW);
        matrix.writeDisplay();
        matrix.blinkRate(1);
  
    } else if (run_mode == 3) {
        matrix.drawBitmap(0, 0, three_bmp, 8, 8, led_color);
        matrix.writeDisplay();
        matrix.blinkRate(1);
    } else if (run_mode == 1) {
        matrix.drawBitmap(0, 0, one_bmp, 8, 8, led_color);
        matrix.writeDisplay();
        matrix.blinkRate(0);
    } else if (run_mode == 2) {
        matrix.drawBitmap(0, 0, two_bmp, 8, 8, led_color);
        matrix.writeDisplay();
        matrix.blinkRate(1);


    } else if (run_mode == 13) {
        matrix.drawBitmap(0, 0, three_follow_bmp, 8, 8, led_color);
        matrix.writeDisplay();
        matrix.blinkRate(1);
    } else if (run_mode == 11) {
        matrix.drawBitmap(0, 0, one_follow_bmp, 8, 8, led_color);
        matrix.writeDisplay();
        matrix.blinkRate(0);
    } else if (run_mode == 12) {
        matrix.drawBitmap(0, 0, two_follow_bmp, 8, 8, led_color);
        matrix.writeDisplay();
        matrix.blinkRate(1);


    } else if (run_mode == 23) {
        matrix.drawBitmap(0, 0, three_furtive_bmp, 8, 8, led_color);
        matrix.writeDisplay();
        matrix.blinkRate(1);
    } else if (run_mode == 21) {
        matrix.drawBitmap(0, 0, one_furtive_bmp, 8, 8, led_color);
        matrix.writeDisplay();
        matrix.blinkRate(0);
    } else if (run_mode == 22) {
        matrix.drawBitmap(0, 0, two_furtive_bmp, 8, 8, led_color);
        matrix.writeDisplay();
        matrix.blinkRate(1);

    } else if (run_mode == 33) {
        matrix.drawBitmap(0, 0, three_play_bmp, 8, 8, led_color);
        matrix.writeDisplay();
        matrix.blinkRate(1);
    } else if (run_mode == 31) {
        matrix.drawBitmap(0, 0, one_play_bmp, 8, 8, led_color);
        matrix.writeDisplay();
        matrix.blinkRate(0);
    } else if (run_mode == 32) {
        matrix.drawBitmap(0, 0, two_play_bmp, 8, 8, led_color);
        matrix.writeDisplay();
        matrix.blinkRate(1);

  
 
    }  else if (run_mode == 5) {
      matrix.drawBitmap(0, 0, line_bmp, 8, 8, LED_YELLOW);
      matrix.writeDisplay();
      matrix.blinkRate(1);
    }  else if (run_mode == 8) {
      matrix.drawBitmap(0, 0, X_bmp, 8, 8, LED_RED);
      matrix.writeDisplay();
      matrix.blinkRate(1);
    }
  }

 
  delay(10);
}


