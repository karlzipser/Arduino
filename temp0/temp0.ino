
#define SERVO_MAX   4000
#define MOTOR_MAX   SERVO_MAX
#define BUTTON_MAX  SERVO_MAX
#define CAMERA_MAX  SERVO_MAX
#define SERVO_MIN   500
#define MOTOR_MIN   SERVO_MIN
#define BUTTON_MIN  SERVO_MIN
#define CAMERA_MIN  SERVO_MIN

#include "PinChangeInterrupt.h" // Adafruit library
#include <Servo.h> // Arduino library

// These come from the radio receiver via three black-red-white ribbons.
#define PIN_SERVO_IN 11
#define PIN_MOTOR_IN 10
#define PIN_BUTTON_IN 9

// These go out to ESC (motor controller) and steer servo via black-red-white ribbons.
#define PIN_SERVO_OUT 12
#define PIN_MOTOR_OUT 8
#define PIN_CAMERA_OUT 5

#define A_PIN_SERVO_FEEDBACK 5


volatile int button_pwm = 1210;
volatile int servo_pwm = 0;
volatile int motor_pwm = 0;

int servo_command_pwm = 0;
int camera_command_pwm = 0;
int motor_command_pwm = 0;
int motor_null_pwm = 1500;
int servo_null_pwm = 1400;

volatile unsigned long int button_prev_interrupt_time = 0;
volatile unsigned long int servo_prev_interrupt_time  = 0;
volatile unsigned long int motor_prev_interrupt_time  = 0;

int max_communication_delay = 100;

long unsigned int servo_command_time;
long unsigned int motor_command_time;
long unsigned int camera_command_time;

Servo servo;
Servo motor; 
Servo camera; 

volatile float encoder_value_1 = 0.0;
volatile float encoder_value_2 = 0.0;

int servos_attached = 0;



void setup()
{
  Serial.begin(115200);
  Serial.setTimeout(5);
  while(Serial.available() > 0) {
    char t = Serial.read();
  }
  
  motor_servo_setup();
  
  encoder_setup();
  //imu_setup();
  //led_setup();
}



void motor_servo_setup()
{ 
  pinMode(PIN_BUTTON_IN, INPUT_PULLUP);
  pinMode(PIN_SERVO_IN, INPUT_PULLUP);
  pinMode(PIN_MOTOR_IN, INPUT_PULLUP);

  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(PIN_BUTTON_IN),
    button_interrupt_service_routine, CHANGE);

  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(PIN_SERVO_IN),
    servo_interrupt_service_routine, CHANGE);

  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(PIN_MOTOR_IN),
    motor_interrupt_service_routine, CHANGE);
  
  while(servo_pwm==0 || motor_pwm==0) {
    delay(200);
    Serial.println("while(servo_pwm==0 || motor_pwm==0)");
  }
  int N = 20;
  int servo_null_pwm__ = 0;
  int motor_null_pwm__ = 0;
  for( int i = 0; i < N; i = i + 1 ) {
    servo_null_pwm__ += servo_pwm;
    motor_null_pwm__ += motor_pwm;
    delay(15);
  }
  servo_null_pwm = servo_null_pwm__ / N;
  motor_null_pwm = motor_null_pwm__ / N;
}


void button_interrupt_service_routine(void) {
  volatile unsigned long int m = micros();
  volatile unsigned long int dt = m - button_prev_interrupt_time;
  button_prev_interrupt_time = m;
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
      servo.writeMicroseconds(servo_command_pwm);
    } else if(servo_null_pwm>0) {
      servo.writeMicroseconds(servo_null_pwm);
    }
  }
  if (dt>CAMERA_MIN && dt<CAMERA_MAX) {
    if(camera_command_pwm>0) {
      camera.writeMicroseconds(camera_command_pwm);
    } else if(servo_null_pwm>0) {
      camera.writeMicroseconds(servo_null_pwm);
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
      motor.writeMicroseconds(motor_command_pwm);
    } else if(motor_null_pwm>0) {
      motor.writeMicroseconds(motor_null_pwm);
    }
  }
}


int led_command = 0;


void loop() {
  int A;
  int num_serial_reads = 3;
  long unsigned int now;
  
  for( int i = 0; i < num_serial_reads; i = i + 1 ) {
    A = Serial.parseInt();
    //A = servo_pwm+5000;
    now = millis();
    if (A>500 && A<3000) {
      servo_command_pwm = A;
      servo_command_time = now;
    } else if (A>=5000 && A<10000) {
      camera_command_pwm = A-5000;
      camera_command_time = now;
    } else if (A>=10000 && A<30000) {
      motor_command_pwm = A-10000;
      motor_command_time = now;
    } else if (A<0) {
      led_command = -A;
    }
  }
  /*
  if(now-servo_command_time > max_communication_delay || now-motor_command_time > max_communication_delay || now-camera_command_time > max_communication_delay) {
    servo_command_pwm = 0;
    motor_command_pwm = 0;
    if(now-servo_command_time > 4*max_communication_delay || now-motor_command_time > 4*max_communication_delay || now-camera_command_time > 4*max_communication_delay) {
      servo.detach(); 
      motor.detach(); 
      camera.detach(); 
      servos_attached = 0;      
    }
  } else{
    if(servos_attached==0) {
      servo.attach(PIN_SERVO_OUT); 
      motor.attach(PIN_MOTOR_OUT); 
      camera.attach(PIN_CAMERA_OUT); 
      servos_attached = 1;
    }
  }
  */
  encoder_loop();
  //imu_loop();
  

  Serial.print("('mse',");
  Serial.print(button_pwm);
  Serial.print(",");
  Serial.print(servo_pwm);
  Serial.print(",");
  Serial.print(motor_pwm);
  Serial.print(",");
  Serial.print(encoder_value_1);
  //Serial.print(",");
  //Serial.print(analogRead(A_PIN_SERVO_FEEDBACK));
  Serial.println(")");

  //led_loop();
  
  delay(10);
}














////////////// ENCODER //////////////////
//PIN's definition
#include "RunningAverage.h"
#define encoder0PinA  2
#define encoder0PinB  3

RunningAverage enc_avg(10);

volatile int encoder0Pos = 0;
volatile boolean PastA = 0;
volatile boolean PastB = 0;
volatile unsigned long int a = 0;
volatile unsigned long int b = 0;
volatile unsigned long int t1 = micros();
volatile unsigned long int t2 = 0;
volatile unsigned long int last_t2 = 0;
volatile unsigned long int dt = 0;


void encoder_setup() 
{
  //Serial.begin(9600);
  pinMode(encoder0PinA, INPUT);
  //turn on pullup resistor
  //digitalWrite(encoder0PinA, HIGH); //ONLY FOR SOME ENCODER(MAGNETIC)!!!! 
  pinMode(encoder0PinB, INPUT); 
  //turn on pullup resistor
  //digitalWrite(encoder0PinB, HIGH); //ONLY FOR SOME ENCODER(MAGNETIC)!!!! 
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
  dt = micros()-t1;
  if (doEncoderAdtSum > 0) {
    enc_avg.addValue(1000.0*1000.0/12.0 * a / doEncoderAdtSum); //6 magnets
    encoder_value_1 = enc_avg.getAverage();
    t1 = micros();
    a = 0;
    doEncoderAdtSum = 0;
  } else if (dt > 100000) {
    enc_avg.clear();
    encoder_value_1 = 0;
    t1 = micros();
    a = 0;
    doEncoderAdtSum = 0;
  }
}

//you may easily modify the code  get quadrature..
//..but be sure this whouldn't let Arduino back! 
volatile float doEncoderAdt = 0.;
void doEncoderA()
{
  t2 = micros();
  a = a + 1;
  doEncoderAdtSum += t2 - last_t2; 
  //doEncoderAdt = float(t2 - last_t2);
  //enc_avg.addValue(62500. / doEncoderAdt);
  //encoder_value_1 = enc_avg.getAverage();
  last_t2 = t2;
}

void doEncoderB()
{
     b += 1;
}
//
///////////////////











//////////////////////////////////////////////////////////////////
//
#include <Wire.h>
#include "Adafruit_LEDBackpack.h"
#include "Adafruit_GFX.h"

Adafruit_BicolorMatrix matrix = Adafruit_BicolorMatrix();


const int STRAIGHT = 1;
const int LEFT = 2;
const int RIGHT = 3;
const int SELECTED = 1;
const int NOT_SELECTED = 2;

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

const int LINE = 15;

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


void led_setup() {
  matrix.begin(0x70);  // pass in the address
  matrix.setRotation(3);
  matrix.blinkRate(0);
  matrix.drawBitmap(0, 0, line_bmp, 8, 8, LED_YELLOW);
  matrix.writeDisplay();
  matrix.blinkRate(1); 
}


long unsigned int start_time = millis();


void led_loop() {

  int led_color = 0;
  int blink_rate = 0;

  unsigned int parsed_int = led_command;

  if (parsed_int > 0) {
      int arrow_direction_or_select = parsed_int/10000;
      int agent = (parsed_int-arrow_direction_or_select*10000)/100;
      int behavioral_mode_or_place = (parsed_int-agent*100-arrow_direction_or_select*10000);

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

    if (behavioral_mode_or_place == LINE) {
      matrix.drawBitmap(0, 0, line_bmp, 8, 8, led_color);
    } else if (behavioral_mode_or_place == CALIBRATE) {
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
int use_gyro = 1;
int use_headings = 1;

uint32_t imu_timer = millis();



int sensor_number = 3;
#define OUTPUT_TO_TX

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


