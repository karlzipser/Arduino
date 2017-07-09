
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

  N6_bmp[] =
  { B11000110,
    B11100110,
    B11110110,
    B11011110,
    B11001110,
    B00000000,
    B11100000,
    B00000000 },


  N5_bmp[] =
  { B11000110,
    B11100110,
    B11110110,
    B11011110,
    B11001110,
    B00000000,
    B11101000,
    B00000000 },

  N3_bmp[] =
  { B00011000,
    B00111100,
    B01111110,
    B00011000,
    B00011000,
    B00000000,
    B00000000,
    B00000000 },

  N7_bmp[] =
  { B11000110,
    B11100110,
    B11110110,
    B11011110,
    B11001110,
    B00000000,
    B11101110,
    B00000000 },


  Freeze_bmp[] =
  { B01111110,
    B01111110,
    B01100000,
    B01111100,
    B01111100,
    B01100000,
    B01100000,
    B00000000},

 

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
  two_bmp[] =
  { B00000000,
    B00000000,
    B00000000,
    B00011000,
    B00011000,
    B01111110,
    B00111100,
    B00011000 },
   one_bmp[] =
  { B00000000,
    B00100000,
    B01100000,
    B11111111,
    B11111111,
    B01100000,
    B00100000,
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



void save_data() {
  matrix.drawBitmap(0, 0, saving_data_bmp, 8, 8, LED_YELLOW);
  matrix.writeDisplay();
  matrix.blinkRate(1);
}

void clear_matrix() {
  matrix.clear();
}

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(5);
  matrix.begin(0x70);  // pass in the address
  matrix.setRotation(3);
  matrix.blinkRate(0);
  
}

long unsigned int start_time = millis();

void loop() {
  if (millis() - start_time < 10000) {
    Serial.println("('GPS2',0,0,0,0,0,0,0,0,0.00000,0.00000,0.00,0.00,0.00,0)"); // dummy data
    delay(100);
  }
  unsigned int parsed_int = Serial.parseInt();
  if (parsed_int > 0) {
      int parsed_mode = parsed_int/10000;
      int parsed_val_1 = (parsed_int-parsed_mode*10000)/100;
      int parsed_val_2 = (parsed_int-parsed_val_1*100-parsed_mode*10000);
      int save_mode = parsed_mode;
      int run_mode = parsed_val_2;
    
  
    
    matrix.clear();
    if (save_mode == 1) {
      save_data();
    }
  
    if (run_mode == 4) {
        matrix.drawBitmap(0, 0, Calibrate_bmp, 8, 8, LED_YELLOW);
        matrix.writeDisplay();
        matrix.blinkRate(1);
  
    } else if (run_mode == 3) {
        matrix.drawBitmap(0, 0, N3_bmp, 8, 8, LED_RED);
        matrix.writeDisplay();
        matrix.blinkRate(1);
        
    } else if (run_mode == 5) {
        matrix.drawBitmap(0, 0, N5_bmp, 8, 8, LED_YELLOW);
        matrix.writeDisplay();
        matrix.blinkRate(0);
  
    } else if (run_mode == 6) {
        matrix.drawBitmap(0, 0, N6_bmp, 8, 8, LED_RED);
        matrix.writeDisplay();
        matrix.blinkRate(0);
  
    } else if (run_mode == 7) {
        matrix.drawBitmap(0, 0, N7_bmp, 8, 8, LED_YELLOW);
        matrix.writeDisplay();
        matrix.blinkRate(0);
  
    } else if (run_mode == 1) {
        matrix.drawBitmap(0, 0, one_bmp, 8, 8, LED_RED);
        matrix.writeDisplay();
        matrix.blinkRate(0);
  
    } else if (run_mode == 2) {
        matrix.drawBitmap(0, 0, two_bmp, 8, 8, LED_RED);
        matrix.writeDisplay();
        matrix.blinkRate(1);
  
    } else if (run_mode == 9) {
        matrix.drawBitmap(0, 0, Freeze_bmp, 8, 8, LED_RED);
        matrix.writeDisplay();
        matrix.blinkRate(1);
  
    }
  }
 
  delay(10);
}


