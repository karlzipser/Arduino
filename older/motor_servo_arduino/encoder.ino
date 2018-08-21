
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
volatile float rate_1 = 0.0;


//you may easily modify the code  get quadrature..
//..but be sure this whouldn't let Arduino back! 
volatile float doEncoderAdt = 0.;
volatile unsigned long int doEncoderAdtSum = 1;
void doEncoderA()
{
  t2 = micros();
  a = a + 1;
  doEncoderAdtSum += t2 - last_t2; 
  //doEncoderAdt = float(t2 - last_t2);
  //enc_avg.addValue(62500. / doEncoderAdt);
  //rate_1 = enc_avg.getAverage();
  last_t2 = t2;
}

void doEncoderB()
{
     b += 1;
}
//
///////////////////


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



void encoder_loop()
{  
  dt = micros()-t1;
  if (doEncoderAdtSum > 0) {
    //enc_avg.addValue(1000.0*1000.0/16.0 * a / doEncoderAdtSum);
    enc_avg.addValue(1000.0*1000.0/12.0 * a / doEncoderAdtSum); //6 magnets
    rate_1 = enc_avg.getAverage();
    t1 = micros();
    a = 0;
    doEncoderAdtSum = 0;
  } else if (dt > 100000) {
    enc_avg.clear();
    rate_1 = 0;
    t1 = micros();
    a = 0;
    doEncoderAdtSum = 0;
  }
}



