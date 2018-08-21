#define USE_MOTOR_SERVO
//#define USE_ENCODER
//#define USE_IMU
#define USE_LED
//#define USE_FLEX


void setup() {
  #ifdef USE_MOTOR_SERVO
    motor_servo_setup();
  #endif
  #ifdef USE_ENCODER
    encoder_setup();
  #endif
  #ifdef USE_IMU
    imu_setup();
  #endif
  #ifdef USE_LED
    led_setup();
  #endif
  #ifdef USE_FLEX
    flex_setup();
  #endif

}


void loop() {
  
  #ifdef USE_MOTOR_SERVO
    motor_servo_loop();
  #endif
  #ifdef USE_ENCODER
    encoder_loop();
  #endif
  #ifdef USE_IMU
    imu_loop();
  #endif
  #ifdef USE_LED
    led_loop();
  #endif
  #ifdef USE_FLEX
    flex_loop();
  #endif

  //GPS_loop();

  delay(10); // loop
}
//
/////////////////////////////////////////











#ifdef USE_MOTOR_SERVO


#include "PinChangeInterrupt.h" // Adafruit library
#include <Servo.h> // Arduino library

/////////////////////
// Servo constants
//
// These define extreme min an max values that should never be broken.
#define SERVO_MAX   4000
#define MOTOR_MAX   SERVO_MAX
#define BUTTON_MAX  SERVO_MAX
#define SERVO_MIN   500
#define MOTOR_MIN   SERVO_MIN
#define BUTTON_MIN  SERVO_MIN
//
/////////////////////

/////////////////////
#define PIN_SERVO_IN 11
#define PIN_MOTOR_IN 10
#define PIN_BUTTON_IN 12
#define PIN_SERVO_OUT 9
#define PIN_MOTOR_OUT 8
#define PIN_LED_OUT 13
//
/////////////////////

volatile int button_pwm = 1210;
volatile int servo_pwm = 0;
volatile int motor_pwm = 0;
volatile int servo_write_pwm = 0;
volatile int motor_write_pwm = 0;
int servo_command_pwm = 0;
int motor_command_pwm = 0;
int motor_null_pwm = 1500;
int servo_null_pwm = 1400;
// These are used to interpret interrupt signals.
volatile unsigned long int button_prev_interrupt_time = 0;
volatile unsigned long int servo_prev_interrupt_time  = 0;
volatile unsigned long int motor_prev_interrupt_time  = 0;
//volatile unsigned long int state_transition_time_ms = 0;

int max_communication_delay = 100;

long unsigned int servo_command_time;
long unsigned int motor_command_time;

Servo servo;
Servo motor; 

volatile float rate_1 = 0.0; // for encoder
//
///////////////////

////////////////////////////////////////
//
int servos_attached = 0;

void motor_servo_setup()
{
  Serial.begin(115200);
  Serial.setTimeout(5);
  
  pinMode(PIN_BUTTON_IN, INPUT_PULLUP);
  pinMode(PIN_SERVO_IN, INPUT_PULLUP);
  pinMode(PIN_MOTOR_IN, INPUT_PULLUP);

  pinMode(PIN_LED_OUT, OUTPUT);

 
  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(PIN_BUTTON_IN),
    button_interrupt_service_routine, CHANGE);
  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(PIN_SERVO_IN),
    servo_interrupt_service_routine, CHANGE);
  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(PIN_MOTOR_IN),
    motor_interrupt_service_routine, CHANGE);

  
  while(Serial.available() > 0) {
    char t = Serial.read();
  }
  
  while(servo_pwm==0 || motor_pwm==0) {
    delay(200);
  }
  servo_null_pwm = servo_pwm;
  motor_null_pwm = motor_pwm;

}











void button_interrupt_service_routine(void) {
  volatile unsigned long int m = micros();
  volatile unsigned long int dt = m - button_prev_interrupt_time;
  button_prev_interrupt_time = m;
  // Human in full control of driving
  if (dt>BUTTON_MIN && dt<BUTTON_MAX) {
    button_pwm = dt;
  }
}







void servo_interrupt_service_routine(void) {
  volatile unsigned long int m = micros();
  volatile unsigned long int dt = m - servo_prev_interrupt_time;
  servo_prev_interrupt_time = m;
  if (dt>SERVO_MIN && dt<SERVO_MAX) {
    servo_pwm = dt;
    if(servo_command_pwm>0) {
      servo_write_pwm = servo_command_pwm;
      servo.writeMicroseconds(servo_write_pwm);
    } else if(servo_null_pwm>0) {
      servo_write_pwm = servo_null_pwm;
      servo.writeMicroseconds(servo_write_pwm);
    }
  } 
}





void motor_interrupt_service_routine(void) {
  volatile unsigned long int m = micros();
  volatile unsigned long int dt = m - motor_prev_interrupt_time;
  motor_prev_interrupt_time = m;
  if (dt>MOTOR_MIN && dt<MOTOR_MAX) {
    motor_pwm = dt;
    if(motor_command_pwm>0) {
      motor_write_pwm = motor_command_pwm;
      motor.writeMicroseconds(motor_write_pwm);
    } else if(motor_null_pwm>0) {
      motor_write_pwm = motor_null_pwm;
      motor.writeMicroseconds(motor_write_pwm);
    }
  }
}


















void motor_servo_loop() {
  
  unsigned int A = Serial.parseInt();
  unsigned int B = Serial.parseInt();

   long unsigned int now = millis();
   
  if (A>500 && A<3000) {
    servo_command_pwm = A;
    servo_command_time = now;
  } else if (A>10000 && A<30000) {
    motor_command_pwm = A-10000;
    motor_command_time = now;
  }
  if (B>500 && B<3000) {
    servo_command_pwm = B;
    servo_command_time = now;
  } else if (B>10000 && B<30000) {
    motor_command_pwm = B-10000;
    motor_command_time = now;
  }
  
  if(now-servo_command_time > max_communication_delay || now-motor_command_time > max_communication_delay) {
    servo_command_pwm = 0;
    motor_command_pwm = 0;
    if(now-servo_command_time > 5*max_communication_delay || now-motor_command_time > 5*max_communication_delay) {
      servo.detach(); 
      motor.detach(); 
      servos_attached = 0;      
    }
  } else{
    if(servos_attached==0) {
      servo.attach(PIN_SERVO_OUT); 
      motor.attach(PIN_MOTOR_OUT); 
      servos_attached = 1;
    }
  }
  

  Serial.print("('mse',");
  Serial.print(button_pwm);
  Serial.print(",");
  Serial.print(servo_pwm);
  Serial.print(",");
  Serial.print(motor_pwm);
  Serial.print(",");
  Serial.print(rate_1);
  Serial.println(")");

  //delay(10); // loop
}
//
/////////////////////////////////////////

#endif







#ifdef USE_ENCODER

////////////// ENCODER //////////////////
//
#include "RunningAverage.h"
#define encoder0PinA  2
#define encoder0PinB  3

RunningAverage enc_avg(10);

volatile int encoder0Pos = 0;
volatile boolean PastA = 0;
volatile boolean PastB = 0;
volatile unsigned long int a = 0;
volatile unsigned long int b = 0;
volatile unsigned long int t_previous = micros();
volatile unsigned long int tA = 0;
volatile unsigned long int tB = 0;
volatile unsigned long int last_tA = 0;
volatile unsigned long int last_tB = 0;
volatile unsigned long int dt = 0;


void encoder_setup() 
{
  pinMode(encoder0PinA, INPUT);
  pinMode(encoder0PinB, INPUT); 
  PastA = (boolean)digitalRead(encoder0PinA); //initial value of channel A;
  PastB = (boolean)digitalRead(encoder0PinB); //and channel B

//To speed up even more, you may define manually the ISRs
// encoder A channel on interrupt 0 (arduino's pin 2)
  attachInterrupt(0, doEncoderA, CHANGE);
// encoder B channel pin on interrupt 1 (arduino's pin 3)
  attachInterrupt(1, doEncoderB, CHANGE); 

  enc_avg.clear();
}

volatile unsigned long int doEncoderAdtSum = 1;

void encoder_loop()
{  
  dt = micros()-t_previous;
  if (doEncoderAdtSum > 0) {
    enc_avg.addValue(1000.0*1000.0/12.0 * a / doEncoderAdtSum); //6 magnets
    rate_1 = enc_avg.getAverage();
    t_previous = micros();
    a = 0;
    doEncoderAdtSum = 0;
  } else if (dt > 100000) {
    enc_avg.clear();
    rate_1 = 0;
    t_previous = micros();
    a = 0;
    doEncoderAdtSum = 0;
  }
  /*
  if (doEncoderBdtSum > 0) {
    enc_avg.addValue(1000.0*1000.0/12.0 * a / doEncoderBdtSum); //6 magnets
    rate_1 = enc_avg.getAverage();
    t_previous = micros();
    b = 0;
    doEncoderBdtSum = 0;
  } else if (dt > 100000) {
    enc_avg.clear();
    rate_1 = 0;
    t_previous = micros();
    b = 0;
    doEncoderBdtSum = 0;
  }
  */
}


volatile float doEncoderAdt = 0.;
volatile float doEncoderBdt = 0.;

void doEncoderA()
{
  tA = micros();
  a = a + 1;
  doEncoderAdtSum += tA - last_tA; 
  last_tA = tA;
}

void doEncoderB()
{
  b += 1;
  /*
  tB = micros();
  b = b + 1;
  doEncoderBdtSum += tB - last_tB; 
  last_tB = tB;
  */
}
//
///////////////////

#endif







#ifdef USE_IMU

//#include <SoftwareSerial.h>
#include <Wire.h>
#include <Adafruit_MMA8451.h>
#include <Adafruit_Sensor.h>

Adafruit_MMA8451 mma = Adafruit_MMA8451();


float ax = 0;
float ay = 0;
float az = 0;
float ctr = 0;

int use_acc = 1;
int use_gyro = 0;
int use_headings = 0;

uint32_t imu_timer = millis();



int sensor_number = 3;
//#define OUTPUT_TO_TX

char* Accs[]={"acc", "ac1", "ac2","ac3","ac4"};
char* Gyros[]={"gyro", "gyr1", "gyr2","gyr3","gyr4"};
char* Heads[]={"head", "hed1", "hed2","hed3","hed4"};

void gyro_setup();
void gyro_loop();
void acc_loop();



void imu_setup()  
{
  Serial.begin(115200);
  
  if (use_gyro) gyro_setup();

  if (use_acc) {
    if (! mma.begin()) {
      while (1);
    }
    mma.setRange(MMA8451_RANGE_2_G);
  } 
}


void imu_loop() {
  if (use_acc) acc_loop_part1();
  if (imu_timer > millis())  imu_timer = millis();
  if (millis() - imu_timer > 10) {
    imu_timer = millis();
    if (use_acc) acc_loop_part2();
    if (use_gyro) gyro_loop();
  }
}
//////////////////////////////////////////////////////



void acc_loop_part1() {
  sensors_event_t event; 
  mma.getEvent(&event);
  ax += event.acceleration.x;
  ay += event.acceleration.y;
  az += event.acceleration.z;
  ctr += 1;
}

void acc_loop_part2() {
  #ifdef OUTPUT_TO_TX
  Serial.print("(");Serial.print(Accs[sensor_number]);Serial.print(",");
  Serial.print(ax/ctr); Serial.print(",");
  Serial.print(ay/ctr); Serial.print(",");
  Serial.print(az/ctr);
  Serial.print(")");
  Serial.println();
  #else
  Serial.print(ax/ctr); Serial.print(" ");
  Serial.print(ay/ctr); Serial.print(" ");
  Serial.print(az/ctr);
  if (use_gyro) Serial.print(" ");
  else Serial.println();
  #endif
  
  ax = 0;
  ay = 0;
  az = 0;
  ctr = 0;
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
  //Serial.println("gyro_loop()");
  
  updateGyroValues();
  if (use_headings) updateHeadings();
  
  #ifdef OUTPUT_TO_TX
  Serial.print("(");Serial.print(Gyros[sensor_number]);Serial.print(",");
  Serial.print(gyroDPS[0]);Serial.print(",");
  Serial.print(gyroDPS[1]);Serial.print(",");
  Serial.print(gyroDPS[2]);Serial.println(")");
  #else
  Serial.print(gyroDPS[0]/10.0);
  Serial.print(" ");
  Serial.print(gyroDPS[1]/10.0);
  Serial.print(" ");
  Serial.print(gyroDPS[2]/10.0);
  Serial.println("");
  #endif
  //printDPS();
  //Serial.print("   -->   ");
  if (use_headings) printHeadings2();
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
  Serial.print("(");Serial.print(Heads[sensor_number]);Serial.print(",");
  Serial.print(',');

  Serial.print(heading[1]);
  Serial.print(',');

  Serial.print(heading[2]);
  if (sensor_number) {Serial.print(",");Serial.print(sensor_number);}
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

#endif












#ifdef USE_LED

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

void led_setup() {
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

void led_loop() {
  if (millis() - start_time < 10000) {
    Serial.println("('GPS2',0,0,0,0,0,0,0,0,0.00000,0.00000,0.00,0.00,0.00,0)"); // dummy data
    //delay(100);   
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
  //delay(10);
}

#endif


#ifdef USE_FLEX

/******************************************************************************
Flex_Sensor_Example.ino
Example sketch for SparkFun's flex sensors
  (https://www.sparkfun.com/products/10264)
Jim Lindblom @ SparkFun Electronics
April 28, 2016

Create a voltage divider circuit combining a flex sensor with a 47k resistor.
- The resistor should connect from A0 to GND.
- The flex sensor should connect from A0 to 3.3V
As the resistance of the flex sensor increases (meaning it's being bent), the
voltage at A0 should decrease.

Development environment specifics:
Arduino 1.6.7
******************************************************************************/

//int flex_pins[] = {A11,A0,A1,A2,A3,A4,A5,A6,A7,A8,A9,A10,A11};
int flex_pins[] = {A6,A7,A8,A9,A10,A11};
int num_pins = 6;
const float VCC = 5.0; // Estimated voltage of Ardunio 5V line
//const float R_DIV = 47500.0; // Measured resistance of 47k resistor
const float R_DIV = 10000.0; // Estimated resistance of 10k resistor

void flex_setup() 
{
  Serial.begin(115200);
  for( int i = 0; i < num_pins; i = i + 1 ) {
        pinMode(flex_pins[i], INPUT);;
     }
}

int flexADC;
float flexV;
float flexR;

void flex_loop() 
{
  for( int i = 0; i < num_pins; i = i + 1 ) {
    flexADC = analogRead(flex_pins[i]);
    flexV = flexADC * VCC / 1023.0;
    flexR = R_DIV * (VCC / flexV - 1.0);
    Serial.print(flexR);Serial.print('\t');
  }
  Serial.println("");
  //delay(10);
}
#endif







/* linking problem
///////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////
///////////// GPS /////////////////////////////////////////////////////////
//    ------> http://www.adafruit.com/products/746
//Adafruit ultimage GPS
//https://learn.adafruit.com/adafruit-ultimate-gps/arduino-wiring
//VIN to +5V
//GND to Ground
//RX to digital 2
//TX to digital 3
#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>
SoftwareSerial mySerial(3, 2);
Adafruit_GPS GPS(&mySerial);
#define GPSECHO  false
boolean usingInterrupt = true; // Determine whether or not to use this
void useInterrupt(boolean); // Func prototype keeps Arduino 0023 happy
void GPS_setup()  
{
  
  //Serial.begin(115200);
  //Serial.println("Adafruit GPS library basic test!");
  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate
  GPS.sendCommand(PGCMD_ANTENNA);
  useInterrupt(true);
  delay(1000);
//  mySerial.println(PMTK_Q_RELEASE);
  
}
SIGNAL(TIMER0_COMPA_vect) {
  char c = GPS.read();
#ifdef UDR0
  if (GPSECHO)
    if (c) UDR0 = c;  
#endif
}
void useInterrupt(boolean v) {
  if (v) {
    OCR0A = 0xAF;
    TIMSK0 |= _BV(OCIE0A);
    usingInterrupt = true;
  } else {
    TIMSK0 &= ~_BV(OCIE0A);
    usingInterrupt = false;
  }
}
uint32_t gps_timer = millis();
void GPS_loop()                     // run over and over again
{
  if (! usingInterrupt) {
    char c = GPS.read();
    if (GPSECHO)
      if (c) Serial.print(c);
  }
  if (GPS.newNMEAreceived()) {
    if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
      return;  // we can fail to parse a sentence in which case we should just wait for another
  }
  if (gps_timer > millis())  gps_timer = millis();
  if (millis() - gps_timer > 250) { 
    gps_timer = millis(); // reset the gps_timer
    if (1) { //GPS.fix) {
      Serial.print("(");
      Serial.print('GPS');
      Serial.print(",");
      Serial.print(GPS.latitudeDegrees, 5);
      Serial.print(", "); 
      Serial.print(GPS.longitudeDegrees, 5);
      Serial.println(")"); 
    }
  }
}
//
///////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////
*/

