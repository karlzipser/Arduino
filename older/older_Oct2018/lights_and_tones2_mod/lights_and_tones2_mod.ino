#define MASTER 1

/////////////////////// TONES ///////////////////////////////
//
#define  c     3830    // 261 Hz 
#define  d     3400    // 294 Hz 
#define  e     3038    // 329 Hz 
#define  f     2864    // 349 Hz 
#define  g     2550    // 392 Hz 
#define  a     2272    // 440 Hz 
#define  b     2028    // 493 Hz 
#define  C     1912    // 523 Hz 
#define  R     0

void pinMode_(int p,int o)
{
  if (MASTER!=0) pinMode(p, o);
}
void digitalWrite_(int p,int o)
{
  if (MASTER!=0) digitalWrite(p,o);
}
/*
void digitalWrite__(int p,int o)
{
  if (MASTER==0) digitalWrite(p,o);
}
*/
int SPEAKER_OUT = 9; // avoid conflict with LED pins below!


long tempo = 10000;
int pause = 1000;
int rest_count = 100;

int tone_ = 0;
int beat = 0;
long duration  = 0;

int mute = 0;

void playTone(int tone_,int duration) {
  if (MASTER==1) return;
  if (mute) return;
  long elapsed_time = 0;
  if (tone_ > 0) { 

    while (elapsed_time < duration) {

      digitalWrite(SPEAKER_OUT,HIGH);
      delayMicroseconds(tone_ / 2);

      digitalWrite(SPEAKER_OUT, LOW);
      delayMicroseconds(tone_ / 2);

      elapsed_time += (tone_);
    } 
  }
  else { 
    for (int j = 0; j < rest_count; j++) {
      delayMicroseconds(duration);  
    }                                
  }
  
  //Serial.println(tone_);                           
}

void play_melody(int melody[],int beats[],int MAX_COUNT) {
  
  if (MASTER==1) return;
  for (int i=0; i<MAX_COUNT; i++) {
    tone_ = melody[i];
    beat = beats[i];

    duration = beat * tempo;

    playTone(tone_,duration); 

    delayMicroseconds(pause);
  }
}
int melody51[] = {  C,  b,  g,  C,  b,   R };
int beats51[]  = { 16, 16, 16,  8,  8,  32 }; 

int melody1929[] = {  C,  b,  g,  C,  b,   e,  R,  C,  c,  g, a, C, R };
int beats1929[]  = { 16, 16, 16,  8,  8,  16, 32, 16, 16, 16, 8, 8, 32 }; 

int melody1930[] = {  C,  b,  g,  C,  b,   e,  R,  C,  c,  g, a, C };
int beats1930[]  = { 16, 16, 16,  8,  8,  16, 32, 16, 16, 16, 8, 8 }; 

int melody4[] = {  c,  b,b,b,b};
int beats4[]  = { 16,  8,8,8,8}; 
int melody3[] = {  d,  b,b,b};
int beats3[]  = { 16,  8,8,8}; 
int melody2[] = {  e,  b,b};
int beats2[]  = { 16,  8,8}; 
int melody1[] = {  f,  b};
int beats1[]  = { 16,  8}; 
int melody0[] = {  f };
int beats0[]  = { 16 }; 

int melody30[] = {  d, f };
int beats30[]  = { 32, 8 };

int melody31[] = {  d,e,f };
int beats31[]  = { 32, 8, 8 };

int melody60[] = {  f, e, d };
int beats60[]  = { 16, 32, 32 };

int melody61[] = {  f,  e };
int beats61[]  = { 16, 32 };
//
//////////////////////////////////////////////////////

/* LED pins
0
1
2
3
4 CENTER purple
5     blue
6     green
7     white

8 RIGHT green
9   yellow
10  red

11 LEFT red
12    yellow
13    green
*/
/////////////////////// LIGHTS ///////////////////////////////
//
int human_driver = 1;
int LEFT_YELLOW = 9;
int RIGHT_YELLOW = 12;
int BLUE = 5;
int WHITE = 7;
int PURPLE = 4;
int GREEN = 6;
int LEFT_RED = 11;
int RIGHT_RED = 10;
int LEFT_GREEN = 13;
int RIGHT_GREEN = 8;
int ZED = WHITE;
int LIDAR = GREEN;
int BAG = BLUE;
//
//////////////////////////////////////////////////////
const int buttonPin = 2;
//const int buttonLED = A0;
long unsigned int button_press_time = millis();
long unsigned int button_press_time_prev = 0;

#include <Wire.h>

void setup()
{
  if (MASTER==1){
    Wire.begin(); // join i2c bus (address optional for master)
    for (int i=4; i<14;++i){
      pinMode_(i, OUTPUT);digitalWrite_(i, LOW);
    }
    pinMode(buttonPin,INPUT);
    //pinMode(buttonLED,OUTPUT);
    Serial.begin(115200);
    Serial.setTimeout(5);
   for (int i = 0; i < 10; i++) {
    while (Serial.available() > 0) {
     char k = Serial.read();
     delay(1);
    }
    delay(1);
   }
    Serial.println("Master setup()");    
  } else if (MASTER==0) {
      Wire.begin(8);                // join i2c bus with address #8
      Wire.onReceive(receiveEvent); // register event
      pinMode(SPEAKER_OUT, OUTPUT);
  }
}


int toggle(int t) {
  if (t==0) t=1;
  else t = 0;
  return t;
}
int ___tic___=1;
long unsigned int ___prev_time___ = millis();
int tic_toc(int delay_time)
{
  long unsigned int now = millis();
  if (now-___prev_time___ > delay_time){
    ___prev_time___ = now;
    ___tic___ = toggle(___tic___);
  }
  return ___tic___;
}


long unsigned int __start_time__;
int __duration__;
int start_stopwatch(int t)
{
  __start_time__ = millis();
  __duration__ = t;
}
int check_stopwatch()
{
  if (millis() - __start_time__ > __duration__) return 1;
  return 0;
}


long unsigned int now_sound = millis();

int button;

int AA;
int lights_out = 0;
void loop() {
  
  int A;

  if (MASTER==1) {
    if (millis()-now_sound > 1000) {
      now_sound = millis();
      Serial.println("('sound',0,0,0)");
    }
  }
  if (MASTER==1) {
    A = Serial.parseInt();
    //Serial.println("here1");
    if (mute == 0) {
      Wire.beginTransmission(8); // transmit to device #8
      Wire.write(A);              // sends one byte
      Wire.endTransmission();
    }
  } else {
    A = AA;
  }
  if (MASTER==1) {
    if (A == 21) {
      lights_out = 0;
    }
    if (lights_out==1) {
      for (int i=0; i<14;++i){
          pinMode(i, OUTPUT);digitalWrite(i, LOW);
      }
      delay(100);
      return;
    }
  }
 
  if (A == 51) { // 51 
    play_melody(melody51,beats51,sizeof(melody51)/2);
  }
  else if (A == 22) {
    lights_out = 1;
  }
  else if (A == 49) { // 1929 tell when can start manual calibration
    play_melody(melody1929,beats1929,sizeof(melody1929)/2);
  }
  else if (A == 100) { // 100 means human driver
    human_driver = 1;
  }
  else if (A == 101) { // 101 means network driver
    human_driver = 0;
  }
  else if (A == 50) { // 1930 indicates new rosbag is being written to
    digitalWrite_(BAG, HIGH);
    if (MASTER==1) {
      start_stopwatch(2500);
    }
    play_melody(melody1930,beats1930,sizeof(melody1930)/2);
  }
  else if (A == 30) { // zed found
    digitalWrite_(ZED, HIGH);
    play_melody(melody30,beats30,sizeof(melody30)/2);
  }
  else if (A == 31) { // lidar
    digitalWrite_(LIDAR, HIGH);
    play_melody(melody31,beats31,sizeof(melody31)/2);
  }
  else if (A == 60) { // 60 indicates no zed found yet
    digitalWrite_(ZED, LOW);
    play_melody(melody60,beats60,sizeof(melody60)/2);
  }
  else if (A == 61) { // no lidar
    digitalWrite_(LIDAR, LOW);
    play_melody(melody61,beats61,sizeof(melody61)/2);
  }
  else if (A == 4) { // button 4 reached
    button = 4;
    Serial.println("A==4");
    digitalWrite_(LEFT_YELLOW, LOW);
    digitalWrite_(RIGHT_YELLOW, LOW);
    digitalWrite_(LEFT_RED, LOW);
    digitalWrite_(RIGHT_RED, LOW);
    digitalWrite_(LEFT_GREEN, LOW);
    digitalWrite_(RIGHT_GREEN, LOW);
    digitalWrite_(PURPLE, HIGH);
    play_melody(melody4,beats4,sizeof(melody4)/2);
  }
  else if (A == 3) { // button 3 reached
    button = 3;
    play_melody(melody0,beats0,sizeof(melody0)/2);
  }
  else if (A == 2) { // button 2 reached
    button = 2;
    digitalWrite_(LEFT_YELLOW, LOW);
    digitalWrite_(RIGHT_YELLOW, LOW);
    play_melody(melody2,beats2,sizeof(melody2)/2);
  }
  else if (A == 1) { // button 1 reached
    button = 1;
    play_melody(melody0,beats0,sizeof(melody0)/2);
  }





  
  if (MASTER==1) {
    if (check_stopwatch()) {
      digitalWrite_(BAG, LOW);
    }
  }
  
  if (button<4) {
    digitalWrite_(PURPLE, LOW);
    if (button==2) {
      if (human_driver) {
        digitalWrite_(LEFT_RED, HIGH);
        digitalWrite_(LEFT_GREEN, LOW);
        digitalWrite_(RIGHT_RED, HIGH);
        digitalWrite_(RIGHT_GREEN, LOW);
      } else {
        digitalWrite_(LEFT_RED, LOW);
        digitalWrite_(LEFT_GREEN, HIGH);
        digitalWrite_(RIGHT_RED, LOW);
        digitalWrite_(RIGHT_GREEN, HIGH);
      }
    } else {
        digitalWrite_(LEFT_RED, LOW);
        digitalWrite_(LEFT_GREEN, LOW);
        digitalWrite_(RIGHT_RED, LOW);
        digitalWrite_(RIGHT_GREEN, LOW);     
    }

    int level = tic_toc(250);

    if (button==1) {
      digitalWrite_(LEFT_YELLOW, LOW);
      digitalWrite_(RIGHT_YELLOW, level);
    } else if (button==3) {
      digitalWrite_(LEFT_YELLOW, level);
      digitalWrite_(RIGHT_YELLOW, LOW);   
    }
    /*
    if (mute==0 && MASTER==0) {
      if (button==1) {;
        digitalWrite(SPEAKER_OUT, level);
        delayMicroseconds(d / 2);
        digitalWrite(SPEAKER_OUT, LOW);
      } else if (button==3) {;
        digitalWrite(SPEAKER_OUT, level);
        delayMicroseconds(d / 2);
        digitalWrite(SPEAKER_OUT, LOW);
      }
    }
    */
    
  }
  /*
  if (MASTER==1) {
    if (mute==1) {
      ;//digitalWrite(buttonLED,HIGH);
    }
    else {
      ;//digitalWrite(buttonLED,LOW);
    }
  }
  int buttonState = digitalRead(buttonPin);
  if (buttonState == HIGH){
    Serial.println("button pressed!");
    button_press_time_prev = button_press_time;
    button_press_time = millis();
    if (button_press_time - button_press_time_prev > 500) {
      mute = toggle(mute);
    }
  }
  */

}


void receiveEvent(int __dummy__) {
  AA = Wire.read();    // receive byte as an integer
}

//EOF
