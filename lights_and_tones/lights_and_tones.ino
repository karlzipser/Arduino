
#define  c     3830    // 261 Hz 
#define  d     3400    // 294 Hz 
#define  e     3038    // 329 Hz 
#define  f     2864    // 349 Hz 
#define  g     2550    // 392 Hz 
#define  a     2272    // 440 Hz 
#define  b     2028    // 493 Hz 
#define  C     1912    // 523 Hz 
#define  R     0

int speakerOut = 9;

long tempo = 10000;
int pause = 1000;
int rest_count = 100;

int tone_ = 0;
int beat = 0;
long duration  = 0;

int mute = 0;

void playTone(int tone_) {
  if (mute) return;
  long elapsed_time = 0;
  if (tone_ > 0) { 

    while (elapsed_time < duration) {

      digitalWrite(speakerOut,HIGH);
      delayMicroseconds(tone_ / 2);

      digitalWrite(speakerOut, LOW);
      delayMicroseconds(tone_ / 2);

      elapsed_time += (tone_);
    } 
  }
  else { 
    for (int j = 0; j < rest_count; j++) {
      delayMicroseconds(duration);  
    }                                
  }                                 
}

void play_melody(int melody[],int beats[],int MAX_COUNT) {
  for (int i=0; i<MAX_COUNT; i++) {
    tone_ = melody[i];
    beat = beats[i];

    duration = beat * tempo;

    playTone(tone_); 

    delayMicroseconds(pause);
    
  }
}


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

int LEFT = 2;
//int CENTER = 3;
int RIGHT = 3;
int BAG = 4;
int ZED = 5;
int LIDAR = 6;

void setup()
{
  pinMode(LEFT, OUTPUT);digitalWrite(LEFT, LOW);
  //pinMode(CENTER, OUTPUT);digitalWrite(CENTER, LOW);
  pinMode(RIGHT, OUTPUT);digitalWrite(RIGHT, LOW);
  pinMode(BAG, OUTPUT);digitalWrite(BAG, LOW);
  pinMode(ZED, OUTPUT);digitalWrite(ZED, LOW);
  pinMode(LIDAR, OUTPUT);digitalWrite(LIDAR, LOW);
  Serial.begin(115200);
  Serial.setTimeout(5);
  Serial.println("setup()");
  pinMode(speakerOut, OUTPUT);
   for (int i = 0; i < 10; i++)
   {
     while (Serial.available() > 0)
     {
       char k = Serial.read();
       delay(1);
     }
     delay(1);
   }
}
int state_left_signal = 0;
int state_right_signal = 0;
int state_center_signal = 0;

long unsigned int now_sound = millis();
long unsigned int now_blink = millis();
int light_off = 0;

void loop() {

  int A;
  

  if (millis()-now_sound > 1000) {
    now_sound = millis();
    Serial.println("('sound',0,0,0)");
  }

  light_off = 0;
  if (millis()-now_blink > 250){
    light_off = 1;
    now_blink = millis();
  }

  A = Serial.parseInt();

  if (A == 1929) { // 1929 tell when can start manual calibration
    play_melody(melody1929,beats1929,sizeof(melody1929)/2);
  }
  else if (A == 1930) { // 1930 indicates new rosbag is being written to
    digitalWrite(BAG, HIGH);
    play_melody(melody1930,beats1930,sizeof(melody1930)/2);
    digitalWrite(BAG, LOW);
  }
  else if (A == 30) { // zed found
    digitalWrite(ZED, HIGH);
    play_melody(melody30,beats30,sizeof(melody30)/2);
  }
  else if (A == 31) { // lidar
    digitalWrite(LIDAR, HIGH);
    play_melody(melody31,beats31,sizeof(melody31)/2);
  }
  else if (A == 60) { // 60 indicates no zed found yet
    digitalWrite(ZED, LOW);
    play_melody(melody60,beats60,sizeof(melody60)/2);
  }
  else if (A == 61) { // no lidar
    digitalWrite(LIDAR, LOW);
    play_melody(melody61,beats61,sizeof(melody61)/2);
  }
  else if (A == 4) { // button 4 reached
    Serial.println("A==4");
    digitalWrite(LEFT, HIGH);
    digitalWrite(RIGHT, HIGH);
    play_melody(melody4,beats4,sizeof(melody4)/2);
  }
  else if (A == 3) { // button 3 reached
    Serial.println("A==3");
    digitalWrite(LEFT, LOW);
    digitalWrite(RIGHT, HIGH);
    play_melody(melody3,beats3,sizeof(melody3)/2);
  }
  else if (A == 2) { // button 2 reached
    Serial.println("A==2");
    digitalWrite(LEFT, HIGH);
    digitalWrite(RIGHT, HIGH);
    play_melody(melody2,beats2,sizeof(melody2)/2);
  }
  else if (A == 1) { // button 1 reached
    Serial.println("A==1");
    digitalWrite(LEFT, HIGH);
    digitalWrite(RIGHT, LOW);
    play_melody(melody1,beats1,sizeof(melody1)/2);
  }

  
  

}


