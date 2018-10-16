
#define SERVO_MAX   4000
#define SERVO_MIN   500


#include "PinChangeInterrupt.h" // Adafruit library
#include <Servo.h> // Arduino library
#define PIN_SERVO_IN 11
#define PIN_SERVO_OUT 9

volatile int servo_pwm = 0;

volatile unsigned long int servo_prev_interrupt_time  = 0;


int servo_command_pwm = 0;
int servo_null_pwm = 1500;

int max_communication_delay = 100;

Servo servo;


int servos_attached = 0;


long unsigned int start_time;

void setup()
{
  Serial.begin(115200);
  Serial.setTimeout(5);
  Serial.println("void setup()");
  /*
   for (int i = 0; i < 10; i++)
   {
     while (Serial.available() > 0)
     {
       char k = Serial.read();
       delay(1);
     }
     delay(1);
   }
   */
  motor_servo_setup();
  start_time = millis();
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
}


void motor_servo_setup()
{ 
  Serial.println("void motor_servo_setup()");
  pinMode(PIN_SERVO_IN, INPUT_PULLUP);
  
  servo.attach(PIN_SERVO_OUT);
  
  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(PIN_SERVO_IN),
    servo_interrupt_service_routine, CHANGE);
  
  while(servo_pwm==0) {
    delay(200);
  }
  
  int N = 20;
  int servo_null_pwm__ = 0;

  for( int i = 0; i < N; i = i + 1 ) {
    servo_null_pwm__ += servo_pwm;

    delay(15);
  }
  servo_null_pwm = servo_null_pwm__ / N;
}




void loop() {

  //Serial.println("void loop()");

  int A;

  long unsigned int now;
  
  //A = Serial.parseInt();

  servo_command_pwm = servo_pwm;
  //servo.writeMicroseconds(servo_command_pwm);
  servo.write(servo_command_pwm/12);
  if (millis()-start_time > 1*1000) {
    Serial.print("('mse',");
    Serial.print(servo_pwm);
    Serial.print(",");
    Serial.print(servo_command_pwm);
    Serial.println(")");
  }
  
  delay(10);
}











//EOF

