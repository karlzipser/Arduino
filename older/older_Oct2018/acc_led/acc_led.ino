#include <SoftwareSerial.h>
#include <Wire.h>
#include <Adafruit_MMA8451.h>
#include <Adafruit_Sensor.h>
#include "constants.h"

Adafruit_MMA8451 mma = Adafruit_MMA8451();


void setup()  
{
  
  Serial.begin(115200);
  gyro_setup();
  led_setup();
  GPS_setup();
  //Serial.println("Adafruit MMA8451 test!");
  if (! mma.begin()) {
    //Serial.println("Couldnt start");
    while (1);
  }
  //Serial.println("MMA8451 found!");
  mma.setRange(MMA8451_RANGE_2_G);
  //Serial.print("Range = "); Serial.print(2 << mma.getRange());  
  //Serial.println("G");
  
}



//////////////////////////////////////////////////////

float ax = 0;
float ay = 0;
float az = 0;
float ctr = 0;

uint32_t timer = millis();
void loop() {
  led_loop();
  GPS_loop();
  //gyro_loop();
/*    
  if (timer > millis())  timer = millis();

  if (millis() - timer > 1000) {
     timer = millis();
  }
  
  sensors_event_t event; 
  mma.getEvent(&event);
  Serial.print("(acc,");
  Serial.print(event.acceleration.x); Serial.print(",");
  Serial.print(event.acceleration.y); Serial.print(",");
  Serial.print(event.acceleration.z); Serial.print(")");
  Serial.println();
  delay(1000/100);
*/
  sensors_event_t event; 
  mma.getEvent(&event);
  ax += event.acceleration.x;
  ay += event.acceleration.y;
  az += event.acceleration.z;
  ctr += 1;


  if (timer > millis())  timer = millis();

  if (millis() - timer > 10) {
    timer = millis();
    Serial.print("('acc',");
    Serial.print(ax/ctr); Serial.print(",");
    Serial.print(ay/ctr); Serial.print(",");
    Serial.print(az/ctr); Serial.print(")");
    //Serial.print(ctr); Serial.print(")");
    Serial.println();
    ax = 0;
    ay = 0;
    az = 0;
    ctr = 0;
    gyro_loop();
  }


}







///////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////
//////////// gyroscope //////////////////////////////////////////////////////////
// PINS: A4,A5
// Baud: 115200
// http://forum.arduino.cc/index.php?topic=147351.0
// http://www.livescience.com/40103-accelerometer-vs-gyroscope.html
//Parallax Gyroscope Module 3-Axis L3G4200D
//https://www.parallax.com/product/27911
//http://forum.arduino.cc/index.php?topic=147351.msg1106879#msg1106879
//VIN to +5V
//GND to Ground
//SCL line to pin A5
//SDA line to pin A4
#include <Wire.h>
#define  CTRL_REG1  0x20
#define  CTRL_REG2  0x21
#define  CTRL_REG3  0x22
#define  CTRL_REG4  0x23
#define  CTRL_REG5  0x24
#define  CTRL_REG6  0x25
int gyroI2CAddr=105;
int gyroRaw[3];                         // raw sensor data, each axis, pretty useless really but here it is.
double gyroDPS[3];                      // gyro degrees per second, each axis
float heading[3]={0.0f};                // heading[x], heading[y], heading [z]
int gyroZeroRate[3];                    // Calibration data.  Needed because the sensor does center at zero, but rather always reports a small amount of rotation on each axis.
int gyroThreshold[3];                   // Raw rate change data less than the statistically derived threshold is discarded.
#define  NUM_GYRO_SAMPLES  50           // As recommended in STMicro doc
#define  GYRO_SIGMA_MULTIPLE  3         // As recommended 
float dpsPerDigit=.00875f;              // for conversion to degrees per second
void gyro_setup() {
  //Serial.begin(115200);
  Wire.begin();
  setupGyro();
  calibrateGyro();
}
void gyro_loop() {
  updateGyroValues();
  updateHeadings();
  Serial.print("(");
  Serial.print(STATE_GYRO);
  Serial.print(",");
  Serial.print(gyroDPS[0]);
  Serial.print(",");
  Serial.print(gyroDPS[1]);
  Serial.print(",");
  Serial.print(gyroDPS[2]);
  Serial.println(")");
  //printDPS();
  //Serial.print("   -->   ");
  printHeadings2();
  //Serial.println();
}
void printDPS()
{
  Serial.print("DPS X: ");
  Serial.print(gyroDPS[0]);
  Serial.print("  Y: ");
  Serial.print(gyroDPS[1]);
  Serial.print("  Z: ");
  Serial.print(gyroDPS[2]);
}
void printHeadings()
{
  Serial.print("Heading X: ");
  Serial.print(heading[0]);
  Serial.print("  Y: ");
  Serial.print(heading[1]);
  Serial.print("  Z: ");
  Serial.print(heading[2]);
}
void printHeadings2()
{
  Serial.print("('head',");
  Serial.print(heading[0]);
  Serial.print(',');

  Serial.print(heading[1]);
  Serial.print(',');

  Serial.print(heading[2]);
  Serial.println(')');
}

void updateHeadings()
{
  float deltaT=getDeltaTMicros();
  for (int j=0;j<3;j++)
    heading[j] -= (gyroDPS[j]*deltaT)/1000000.0f;
}
unsigned long getDeltaTMicros()
{
  static unsigned long lastTime=0;
  unsigned long currentTime=micros();
  unsigned long deltaT=currentTime-lastTime;
  if (deltaT < 0.0)
     deltaT=currentTime+(4294967295-lastTime);
  lastTime=currentTime;
  return deltaT;
}
void testCalibration()
{
  calibrateGyro();
  for (int j=0;j<3;j++)
  {
    Serial.print(gyroZeroRate[j]);
    Serial.print("  ");
    Serial.print(gyroThreshold[j]);
    Serial.print("  ");  
  }
  Serial.println();
  return; 
}
void setupGyro()
{
  gyroWriteI2C(CTRL_REG1, 0x1F);        // Turn on all axes, disable power down
  gyroWriteI2C(CTRL_REG3, 0x08);        // Enable control ready signal
  setGyroSensitivity500();
  delay(100);
}
void calibrateGyro()
{
  long int gyroSums[3]={0};
  long int gyroSigma[3]={0};
  for (int i=0;i<NUM_GYRO_SAMPLES;i++)
  {
    updateGyroValues();
    for (int j=0;j<3;j++)
    {
      gyroSums[j]+=gyroRaw[j];
      gyroSigma[j]+=gyroRaw[j]*gyroRaw[j];
    }
  }
  for (int j=0;j<3;j++)
  {
    int averageRate=gyroSums[j]/NUM_GYRO_SAMPLES;
    gyroZeroRate[j]=averageRate;
    gyroThreshold[j]=sqrt((double(gyroSigma[j]) / NUM_GYRO_SAMPLES) - (averageRate * averageRate)) * GYRO_SIGMA_MULTIPLE;    
  }
}
void updateGyroValues() {
  while (!(gyroReadI2C(0x27) & B00001000)){}      // Without this line you will get bad data occasionally
  int reg=0x28;
  for (int j=0;j<3;j++)
  {
    gyroRaw[j]=(gyroReadI2C(reg) | (gyroReadI2C(reg+1)<<8));
    reg+=2;
  }
  int deltaGyro[3];
  for (int j=0;j<3;j++)
  {
    deltaGyro[j]=gyroRaw[j]-gyroZeroRate[j];      // Use the calibration data to modify the sensor value.
    if (abs(deltaGyro[j]) < gyroThreshold[j])
      deltaGyro[j]=0;
    gyroDPS[j]= dpsPerDigit * deltaGyro[j];      // Multiply the sensor value by the sensitivity factor to get degrees per second.
  }
}
void setGyroSensitivity250(void)
{
  dpsPerDigit=.00875f;
  gyroWriteI2C(CTRL_REG4, 0x80);        // Set scale (250 deg/sec)
}
void setGyroSensitivity500(void)
{
  dpsPerDigit=.0175f;
  gyroWriteI2C(CTRL_REG4, 0x90);        // Set scale (500 deg/sec)
}
void setGyroSensitivity2000(void)
{
  dpsPerDigit=.07f;
  gyroWriteI2C(CTRL_REG4,0xA0); 
}
int gyroReadI2C (byte regAddr) {
  Wire.beginTransmission(gyroI2CAddr);
  Wire.write(regAddr);
  Wire.endTransmission();
  Wire.requestFrom(gyroI2CAddr, 1);
  while(!Wire.available()) {};
  return (Wire.read());
}
int gyroWriteI2C( byte regAddr, byte val){
  Wire.beginTransmission(gyroI2CAddr);
  Wire.write(regAddr);
  Wire.write(val);
  Wire.endTransmission();
}
//
//////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////











///////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////

#include <Wire.h>
#include "Adafruit_LEDBackpack.h"
#include "Adafruit_GFX.h"

Adafruit_BicolorMatrix matrix0 = Adafruit_BicolorMatrix();
Adafruit_BicolorMatrix matrix1 = Adafruit_BicolorMatrix();

// [orientation][blink][color][_,symbol]
// e.g., 12304 = straight, blink=2, yellow, play

const int STRAIGHT = 1;
const int LEFT = 2;
const int RIGHT = 3;

const int BLINK_0 = 0;
const int BLINK_1 = 1;
const int BLINK_2 = 2;

const int RED = 1;
const int GREEN = 2;
const int YELLOW = 3;

const int DIRECT = 1;
const int FOLLOW = 2;
const int FURTIVE = 3;
const int PLAY = 4;
const int CALIBRATE = 5;
const int DRIVE_MODE = 6;
const int SELECT_MODE = 7;
const int LOCAL = 8;
const int HOME = 9;
const int TILDEN = 10;
const int CAMPUS = 11;
const int ARENA = 12;
const int OTHER = 13;
//const int AGENT = 14;
const int LINE = 15;
const int LINE_BUTTON_4 = 16;
const int HUMAN = 17;
const int NETWORK = 18;
const int X = 19;

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
human_bmp[] =
{ B00000000,
  B00000000,
  B11111110,
  B00010000,
  B00010000,
  B11111110,
  B00000000,
  B00000000 },
network_bmp[] =
{ B00000000,
  B00000000,
  B11111110,
  B00110000,
  B1100000,
  B11111110,
  B00000000,
  B00000000 },
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
line_4_bmp[] =
{ B00000000,
  B00000001,
  B00000001,
  B11111101,
  B00100001,
  B11100001,
  B00000001,
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



void led_setup() {
  Serial.begin(115200);
  Serial.setTimeout(5);
  matrix0.begin(0x70);  // pass in the address
  matrix0.setRotation(3);//0/1/2
  matrix0.blinkRate(0);
  matrix0.drawBitmap(0, 0, line_bmp, 8, 8, LED_YELLOW);
  matrix0.writeDisplay();
  matrix0.blinkRate(0);

  matrix1.begin(0x71);  // pass in the address
  matrix1.setRotation(0);//3/1/2
  matrix1.drawBitmap(0, 0, line_bmp, 8, 8, LED_RED);
  matrix1.writeDisplay();
  matrix1.blinkRate(0);
  //GPS_setup();
}


long unsigned int millis_counter = millis();
int num_bag_files = 0;
int num_arduinos = 0;
int wifi_status = 0;
int GPS_status = 0;


void led_loop() {

  if (millis()-millis_counter > 4000) {millis_counter = millis();}
  int led_color = 0;
  int blink_rate = 0;
  int parsed_int = Serial.parseInt();

  if (parsed_int > 0) {
      int orientation = (parsed_int)/10000;
      int blink =       (parsed_int-10000*orientation)/1000;
      int color =       (parsed_int-10000*orientation-1000*blink)/100;
      int symbol =      (parsed_int-10000*orientation-1000*blink-100*color)/1;

    if (blink == BLINK_0) {
      blink_rate = 0;
    } else if (blink == BLINK_1) {
      blink_rate = 1;
    } else if (blink == BLINK_2) {
      blink_rate = 2;
    }

    if (color == RED) {
      led_color = LED_RED;}
    else if (color == GREEN) {
      led_color = LED_GREEN;}
    else if(color == YELLOW)
      {led_color = LED_YELLOW;}

    matrix0.clear();

    if (symbol == LINE) {
      matrix0.drawBitmap(0, 0, line_bmp, 8, 8, led_color);
    } else if (symbol == CALIBRATE) {
      matrix0.drawBitmap(0, 0, Calibrate_bmp, 8, 8, led_color);
    } else if (symbol == DRIVE_MODE) {
      matrix0.drawBitmap(0, 0, drive_mode_bmp, 8, 8, led_color);
    } else if (symbol == SELECT_MODE) {
      matrix0.drawBitmap(0, 0, select_mode_bmp, 8, 8, led_color);
    } else if (symbol == LINE_BUTTON_4) {
      matrix0.drawBitmap(0, 0, line_4_bmp, 8, 8, led_color);
    } else if (symbol == HUMAN) {
      matrix0.drawBitmap(0, 0, human_bmp, 8, 8, led_color);
    } else if (symbol == NETWORK) {
      matrix0.drawBitmap(0, 0, network_bmp, 8, 8, led_color);
    } else if (symbol == X) {
      matrix0.drawBitmap(0, 0, X_bmp, 8, 8, led_color);
    } else if (symbol == DIRECT) {
        if (orientation == STRAIGHT) {
          matrix0.drawBitmap(0, 0, direct_straight_bmp, 8, 8, led_color);
        } else if (orientation == LEFT) {
          matrix0.drawBitmap(0, 0, direct_left_bmp, 8, 8, led_color);
        } else if (orientation == RIGHT) {
          matrix0.drawBitmap(0, 0, direct_right_bmp, 8, 8, led_color);
        }
    } else if (symbol == FOLLOW) {
        if (orientation == STRAIGHT) {
          matrix0.drawBitmap(0, 0, follow_straight_bmp, 8, 8, led_color);
        } else if (orientation == LEFT) {
          matrix0.drawBitmap(0, 0, follow_left_bmp, 8, 8, led_color);
        } else if (orientation == RIGHT) {
          matrix0.drawBitmap(0, 0, follow_right_bmp, 8, 8, led_color);
        }
    } else if (symbol == FURTIVE) {
        if (orientation == STRAIGHT) {
          matrix0.drawBitmap(0, 0, furtive_straight_bmp, 8, 8, led_color);
        } else if (orientation == LEFT) {
          matrix0.drawBitmap(0, 0, furtive_left_bmp, 8, 8, led_color);
        } else if (orientation == RIGHT) {
          matrix0.drawBitmap(0, 0, furtive_right_bmp, 8, 8, led_color);
        }
    } else if (symbol == PLAY) {
        if (orientation == STRAIGHT) {
          matrix0.drawBitmap(0, 0, play_straight_bmp, 8, 8, led_color);
        } else if (orientation == LEFT) {
          matrix0.drawBitmap(0, 0, play_left_bmp, 8, 8, led_color);
        } else if (orientation == RIGHT) {
          matrix0.drawBitmap(0, 0, play_right_bmp, 8, 8, led_color);
        }

    } else if (symbol == LOCAL) {
          matrix0.drawBitmap(0, 0, local_bmp, 8, 8, led_color);
    } else if (symbol == HOME) {
          matrix0.drawBitmap(0, 0, home_bmp, 8, 8, led_color);
    } else if (symbol == TILDEN) {
          matrix0.drawBitmap(0, 0, Tilden_bmp, 8, 8, led_color);
    } else if (symbol == CAMPUS) {
          matrix0.drawBitmap(0, 0, campus_bmp, 8, 8, led_color);
    } else if (symbol == ARENA) {
          matrix0.drawBitmap(0, 0, arena_bmp, 8, 8, led_color);
    } else if (symbol == OTHER) {
          matrix0.drawBitmap(0, 0, other_bmp, 8, 8, led_color);

     matrix0.drawBitmap(0, 0, agent_bmp, 8, 8, led_color);
    }

    matrix0.writeDisplay();
    matrix0.blinkRate(blink_rate);

  } else if (-parsed_int >= 10000) {
    
    parsed_int =          -parsed_int;
    GPS_status = (parsed_int)/10000;
    num_arduinos = (parsed_int-10000*GPS_status)/1000;
    num_bag_files =       (parsed_int-10000*GPS_status-1000*num_arduinos);
    if (num_bag_files < 500) wifi_status = 0;
    else {
      num_bag_files -= 500;
      wifi_status = 1;
    }
    if (GPS_status==1) GPS_status = 0;
    else if (GPS_status==2) GPS_status = 1;

    /*
    Serial.println("[");
    Serial.println(parsed_int);
    Serial.println(wifi_status);
    Serial.println(GPS_status);
    Serial.println(num_arduinos);
    Serial.println(num_bag_files);
    //Serial.println(millis()-millis_counter);
    Serial.println("]");
    // good test number: -24565
    */
    
    if (millis()-millis_counter < 1000) {
      //Serial.println("num_bag_files");
      matrix1.clear();
      if (parsed_int == 11119){
        matrix1.drawBitmap(0, 0, X_bmp, 8, 8, LED_RED);
      } else {
        int ctr = 0;
        for (int x=0;x<8;x++){
          for (int y=0;y<8;y++){
            if (ctr < num_bag_files) {
              if (ctr<64) {
                matrix1.drawPixel(x, y, LED_GREEN);
              }
              ctr++;
            }
          }
        }
      }
      if (num_bag_files > 64) {
        int ctr=0;
        for (int x=0;x<8;x++){
          for (int y=0;y<8;y++){
            if (ctr < num_bag_files) {
                matrix1.drawPixel(x, y, LED_RED);
            }
            ctr++;
          }
        } 
      } 

    }  else if (millis()-millis_counter < 2000) {
      //Serial.println("GPS_status");
      if (GPS_status) {
        matrix1.setTextColor(LED_GREEN);
      } else {
        matrix1.setTextColor(LED_RED);
      }
      matrix1.clear();
      matrix1.setCursor(0,0);
      matrix1.print("G");

    } else if (millis()-millis_counter < 3000) {
      //Serial.println("wifi_status");

      if (wifi_status) {
        matrix1.setTextColor(LED_GREEN);
      } else {
        matrix1.setTextColor(LED_RED);
      }    
      matrix1.clear();  
      matrix1.setCursor(0,0);
      matrix1.print("W");

    } else if (millis()-millis_counter < 4000) {
      //Serial.println("num_arduinos");
      matrix1.setTextColor(LED_YELLOW);
      matrix1.clear();
      matrix1.setCursor(0,0);
      matrix1.print(num_arduinos); 
    } else {
        millis_counter = millis();
    }
    matrix1.writeDisplay();
    matrix1.blinkRate(0);
  }
  //GPS_loop();
  delay(0);
}














// Test code for Adafruit GPS modules using MTK3329/MTK3339 driver
//
// This code shows how to listen to the GPS module in an interrupt
// which allows the program to have more 'freedom' - just parse
// when a new NMEA sentence is available! Then access data when
// desired.
//
// Tested and works great with the Adafruit Ultimate GPS module
// using MTK33x9 chipset
//    ------> http://www.adafruit.com/products/746
// Pick one up today at the Adafruit electronics shop 
// and help support open source hardware & software! -ada

#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>

// If you're using a GPS module:
// Connect the GPS Power pin to 5V
// Connect the GPS Ground pin to ground
// If using software serial (sketch example default):
//   Connect the GPS TX (transmit) pin to Digital 3
//   Connect the GPS RX (receive) pin to Digital 2
// If using hardware serial (e.g. Arduino Mega):
//   Connect the GPS TX (transmit) pin to Arduino RX1, RX2 or RX3
//   Connect the GPS RX (receive) pin to matching TX1, TX2 or TX3

// If you're using the Adafruit GPS shield, change 
// SoftwareSerial mySerial(3, 2); -> SoftwareSerial mySerial(8, 7);
// and make sure the switch is set to SoftSerial
SoftwareSerial mySerial(3, 2);
// If using software serial, keep this line enabled
// (you can change the pin numbers to match your wiring):
//SoftwareSerial mySerial(9,8);
//SoftwareSerial mySerial(5,4);

// If using hardware serial (e.g. Arduino Mega), comment out the
// above SoftwareSerial line, and enable this line instead
// (you can change the Serial number to match your wiring):

//HardwareSerial mySerial = Serial1;


Adafruit_GPS GPS(&mySerial);


// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
// Set to 'true' if you want to debug and listen to the raw GPS sentences. 
#define GPSECHO  false

// this keeps track of whether we're using the interrupt
// off by default!
boolean usingInterrupt = false;
void useInterrupt(boolean); // Func prototype keeps Arduino 0023 happy

void GPS_setup()  
{
    
  // connect at 115200 so we can read the GPS fast enough and echo without dropping chars
  // also spit it out
  Serial.begin(115200);
  //Serial.println("Adafruit GPS library basic test!");

  // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
  GPS.begin(9600);
  
  // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  // uncomment this line to turn on only the "minimum recommended" data
  //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  // For parsing data, we don't suggest using anything but either RMC only or RMC+GGA since
  // the parser doesn't care about other sentences at this time
  
  // Set the update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate
  // For the parsing code to work nicely and have time to sort thru the data, and
  // print it out we don't suggest using anything higher than 1 Hz

  // Request updates on antenna status, comment out to keep quiet
  GPS.sendCommand(PGCMD_ANTENNA);

  // the nice thing about this code is you can have a timer0 interrupt go off
  // every 1 millisecond, and read data from the GPS for you. that makes the
  // loop code a heck of a lot easier!
  useInterrupt(true);

  delay(1000);
  // Ask for firmware version
  mySerial.println(PMTK_Q_RELEASE);
  //LED_setup();
}


// Interrupt is called once a millisecond, looks for any new GPS data, and stores it
SIGNAL(TIMER0_COMPA_vect) {
  char c = GPS.read();
  // if you want to debug, this is a good time to do it!
#ifdef UDR0
  if (GPSECHO)
    if (c) UDR0 = c;  
    // writing direct to UDR0 is much much faster than Serial.print 
    // but only one character can be written at a time. 
#endif
}

void useInterrupt(boolean v) {
  if (v) {
    // Timer0 is already used for millis() - we'll just interrupt somewhere
    // in the middle and call the "Compare A" function above
    OCR0A = 0xAF;
    TIMSK0 |= _BV(OCIE0A);
    usingInterrupt = true;
  } else {
    // do not call the interrupt function COMPA anymore
    TIMSK0 &= ~_BV(OCIE0A);
    usingInterrupt = false;
  }
}

uint32_t gtimer = millis();
void GPS_loop()                     // run over and over again
{
  // in case you are not using the interrupt above, you'll
  // need to 'hand query' the GPS, not suggested :(
  if (! usingInterrupt) {
    // read data from the GPS in the 'main loop'
    char c = GPS.read();
    // if you want to debug, this is a good time to do it!
    if (GPSECHO)
      if (c) Serial.print(c);
  }
  
  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences! 
    // so be very wary if using OUTPUT_ALLDATA and trytng to print out data
    //Serial.println(GPS.lastNMEA());   // this also sets the newNMEAreceived() flag to false
  
    if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
      return;  // we can fail to parse a sentence in which case we should just wait for another
  }

  // if millis() or gtimer wraps around, we'll just reset it
  if (gtimer > millis())  gtimer = millis();

  // approximately every 2 seconds or so, print out the current stats
  if (millis() - gtimer > 500) { 
    gtimer = millis(); // reset the timer
    Serial.print("('GPS3',");
    Serial.print(GPS.latitudeDegrees, 5); Serial.print(","); 
    Serial.print(GPS.longitudeDegrees, 5);Serial.print(",");
    Serial.print(GPS.speed);              Serial.print(",");
    Serial.print(GPS.angle);              Serial.print(",");
    Serial.print(GPS.altitude);           Serial.print(",");
    Serial.print((int)GPS.fixquality);    Serial.print(",");
    Serial.print((int)GPS.satellites);
    Serial.println(")");
  }
  //LED_loop();
}



