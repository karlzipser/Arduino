
///////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////

#include <Wire.h>
#include "Adafruit_LEDBackpack.h"
#include "Adafruit_GFX.h"

Adafruit_BicolorMatrix matrix = Adafruit_BicolorMatrix();


const int STRAIGHT = 1;
const int LEFT = 2;
const int RIGHT = 3;
const int SELECTED = 1;
const int NOT_SELECTED = 0;

const int DIRECT = 1;
const int FOLLOW = 2;
const int FURTIVE = 3;
const int PLAY = 4;

const int CALIBRATE = 5;
const int DRIVE_MODE = 15;
const int SELECT_MODE = 16;


const int HUMAN = 6;
const int NETWORK = 7;

const int LOCAL = 8;
const int HOME = 9;
const int TILDEN = 10;
const int CAMPUS = 11;
const int ARENA = 12;
const int OTHER = 13;

const int AGENT = 14;

static const uint8_t PROGMEM //type for bitmaps
Calibrate_bmp[] =
{ B00000000,
  B00101000,
  B01101100,
  B11000110,
  B11000110,
  B11000110,
  B01111100,
  B00111000 },
drive_mode_bmp[] =
{ B00000000,
  B00111000,
  B01111100,
  B11000110,
  B11000111,
  B11000111,
  B11111111,
  B00000000 },
select_mode_bmp[] =
{ B1100110,
  B1101111,
  B11011111,
  B11011011,
  B11011011,
  B11111011,
  B01111011,
  B00110000 },
_bitmap[] =
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
direct_straight_bmp[] =
{ B00000000,
  B00100000,
  B01100000,
  B11111100,
  B11111100,
  B01100000,
  B00100000,
  B00000000 },
direct_left_bmp[] =
{ B00000000,
  B00000000,
  B00000000,
  B00011000,
  B00011000,
  B01111110,
  B00111100,
  B00011000 },
direct_right_bmp[] =
{ B00011000,
  B00111100,
  B01111110,
  B00011000,
  B00011000,
  B00000000,
  B00000000,
  B00000000 },
follow_straight_bmp[] =
{ B10000000,
  B10001000,
  B10011000,
  B10111111,
  B10111111,
  B10011000,
  B10001000,
  B10000000 },
follow_left_bmp[] =
{ B00000000,
  B00011000,
  B00011000,
  B01111110,
  B00111100,
  B00011000,
  B00000000,
  B11111111
  },
follow_right_bmp[] =
{ B11111111,
  B00000000,
  B00011000,
  B00111100,
  B01111110,
  B00011000,
  B00011000,
  B00000000
  },
furtive_straight_bmp[] =
{ B00000000,
  B00100000,
  B01000000,
  B11111100,
  B01000000,
  B00100000,
  B00000000,
  B00000000 },
furtive_left_bmp[] =
{ B00000000,
  B00000000,
  B00000000,
  B00001000,
  B00001000,
  B00101010,
  B00011100,
  B00001000 },
furtive_right_bmp[] =
{ B00001000,
  B00011100,
  B00101010,
  B00001000,
  B00001000,
  B00000000,
  B00000000,
  B00000000 },
play_straight_bmp[] =
{ B00010000,
  B00110000,
  B01111111,
  B11111111,
  B11111111,
  B01111111,
  B00110000,
  B00010000 },
play_left_bmp[] =
{ B00111100,
  B00111100,
  B00111100,
  B00111100,
  B11111111,
  B01111110,
  B00111100,
  B00011000 },
play_right_bmp[] =
{ B00011000,
  B00111100,
  B01111110,
  B11111111,
  B00111100,
  B00111100,
  B00111100,
  B00111100 },
home_bmp[] =
{ B00000000,
  B11111111,
  B11111111,
  B00110000,
  B00110000,
  B11111111,
  B11111111,
  B00000000 },
Tilden_bmp[] =
{ B00000000,
  B11000000,
  B11000000,
  B11111111,
  B11111111,
  B11000000,
  B11000000,
  B00000000 },
local_bmp[] =
{ B00000000,
  B00000011,
  B00000011,
  B00000011,
  B00000011,
  B11111111,
  B11111111,
  B00000000 },
campus_bmp[] =
{ B00000000,
  B11111110,
  B11111111,
  B00000011,
  B00000011,
  B11111111,
  B11111110,
  B00000000 },
arena_bmp[] =
{ B00000000,
  B01111111,
  B11111111,
  B11011000,
  B11011000,
  B11111111,
  B01111111,
  B00000000 },
other_bmp[] =
{ B00000000,
  B01111110,
  B11111111,
  B11000011,
  B11000011,
  B11111111,
  B01111110,
  B00000000 },
agent_bmp[] =
{ B00000000,
  B01111110,
  B11111111,
  B11011011,
  B11011011,
  B11011111,
  B01101110,
  B00000000 },
line_bmp[] =
{ B00000000,
  B00000001,
  B00000001,
  B00000001,
  B00000001,
  B00000001,
  B00000001,
  B00000000 };


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

  int led_color = 0;
  int blink_rate = 0;

  if (millis() - start_time < 10000) {
    Serial.println("('GPS2',0,0,0,0,0,0,0,0,0.00000,0.00000,0.00,0.00,0.00,0)"); // dummy data
    delay(100);   
  }

  unsigned int parsed_int = Serial.parseInt();

  if (parsed_int > 0) {
      int arrow_direction_or_select = parsed_int/10000;
      int agent = (parsed_int-arrow_direction_or_select*10000)/100;
      int behavioral_mode_or_place = (parsed_int-agent*100-arrow_direction_or_select*10000);

    //Serial.print(arrow_direction_or_select);
    //Serial.print(",");
    //Serial.print(agent);
    //Serial.print(",");
    //Serial.println(behavioral_mode_or_place);

    if (agent == HUMAN) {
      led_color = LED_RED;}
    else if (agent == NETWORK) {
      led_color = LED_GREEN;}
    else {led_color = LED_YELLOW;}

    if (arrow_direction_or_select == STRAIGHT) {
      blink_rate = 0;
    } else if (arrow_direction_or_select == LEFT) {
      blink_rate = 1;
    } else if (arrow_direction_or_select == RIGHT) {
      blink_rate = 1;
    } else if (arrow_direction_or_select == SELECTED) {
      blink_rate = 0;
    } else if (arrow_direction_or_select == NOT_SELECTED) {
      blink_rate = 1;
    }
    matrix.clear();

    if (behavioral_mode_or_place == CALIBRATE) {
      matrix.drawBitmap(0, 0, Calibrate_bmp, 8, 8, led_color);
    } else if (behavioral_mode_or_place == DRIVE_MODE) {
      matrix.drawBitmap(0, 0, drive_mode_bmp, 8, 8, led_color);
    } else if (behavioral_mode_or_place == SELECT_MODE) {
      matrix.drawBitmap(0, 0, select_mode_bmp, 8, 8, led_color);

    } else if (behavioral_mode_or_place == DIRECT) {
        if (arrow_direction_or_select == STRAIGHT) {
          matrix.drawBitmap(0, 0, direct_straight_bmp, 8, 8, led_color);
        } else if (arrow_direction_or_select == LEFT) {
          matrix.drawBitmap(0, 0, direct_left_bmp, 8, 8, led_color);
        } else if (arrow_direction_or_select == RIGHT) {
          matrix.drawBitmap(0, 0, direct_right_bmp, 8, 8, led_color);
        }
    } else if (behavioral_mode_or_place == FOLLOW) {
        if (arrow_direction_or_select == STRAIGHT) {
          matrix.drawBitmap(0, 0, follow_straight_bmp, 8, 8, led_color);
        } else if (arrow_direction_or_select == LEFT) {
          matrix.drawBitmap(0, 0, follow_left_bmp, 8, 8, led_color);
        } else if (arrow_direction_or_select == RIGHT) {
          matrix.drawBitmap(0, 0, follow_right_bmp, 8, 8, led_color);
        }
    } else if (behavioral_mode_or_place == FURTIVE) {
        if (arrow_direction_or_select == STRAIGHT) {
          matrix.drawBitmap(0, 0, furtive_straight_bmp, 8, 8, led_color);
        } else if (arrow_direction_or_select == LEFT) {
          matrix.drawBitmap(0, 0, furtive_left_bmp, 8, 8, led_color);
        } else if (arrow_direction_or_select == RIGHT) {
          matrix.drawBitmap(0, 0, furtive_right_bmp, 8, 8, led_color);
        }
    } else if (behavioral_mode_or_place == PLAY) {
        if (arrow_direction_or_select == STRAIGHT) {
          matrix.drawBitmap(0, 0, play_straight_bmp, 8, 8, led_color);
        } else if (arrow_direction_or_select == LEFT) {
          matrix.drawBitmap(0, 0, play_left_bmp, 8, 8, led_color);
        } else if (arrow_direction_or_select == RIGHT) {
          matrix.drawBitmap(0, 0, play_right_bmp, 8, 8, led_color);
        }

    } else if (behavioral_mode_or_place == LOCAL) {
          matrix.drawBitmap(0, 0, local_bmp, 8, 8, led_color);
    } else if (behavioral_mode_or_place == HOME) {
          matrix.drawBitmap(0, 0, home_bmp, 8, 8, led_color);
    } else if (behavioral_mode_or_place == TILDEN) {
          matrix.drawBitmap(0, 0, Tilden_bmp, 8, 8, led_color);
    } else if (behavioral_mode_or_place == CAMPUS) {
          matrix.drawBitmap(0, 0, campus_bmp, 8, 8, led_color);
    } else if (behavioral_mode_or_place == ARENA) {
          matrix.drawBitmap(0, 0, arena_bmp, 8, 8, led_color);
    } else if (behavioral_mode_or_place == OTHER) {
          matrix.drawBitmap(0, 0, other_bmp, 8, 8, led_color);

    } else if (behavioral_mode_or_place == AGENT) {
          matrix.drawBitmap(0, 0, agent_bmp, 8, 8, led_color);
    }

    matrix.writeDisplay();
    matrix.blinkRate(blink_rate);

  }
 
  delay(10);
}


