
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

void playTone(int tone_) {
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


int melody1929[] = {  C,  b,  g,  C,  b,   e,  R,  C,  c,  g, a, C };
int beats1929[]  = { 16, 16, 16,  8,  8,  16, 32, 16, 16, 16, 8, 8 }; 

int melody4[] = {  c,  b,b,b,b};
int beats4[]  = { 16,  8,8,8,8}; 
int melody3[] = {  d,  b,b,b};
int beats3[]  = { 16,  8,8,8}; 
int melody2[] = {  e,  b,b};
int beats2[]  = { 16,  8,8}; 
int melody1[] = {  f,  b};
int beats1[]  = { 16,  8}; 





LEFT = 2
CENTER = 3
RIGHT = 4

void setup()
{
  pinMode(LEFT, OUTPUT);
  pinMode(CENTER, OUTPUT);
  pinMode(RIGHT, OUTPUT);

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

long unsigned int now_sound = millis();
long unsigned int now_blink = millis();
int light_on = 1;

  
}

// the loop function runs over and over again forever
void loop() {
  digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(1000);                       // wait for a second
  digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
  delay(1000);                       // wait for a second
}

void loop() {

  int A;

  if (millis()-now_sound > 500) {
    now_sound = millis();
    Serial.println("('sound',0,0,0)");
  }
  if (millis()-now_blink> 50) {
    now_blink = millis();
    if (light_on) light_on = 0;
    else light_on = 1;
  }
  A = Serial.parseInt();
  if (A == 1929) {
    play_melody(melody1929,beats1929,sizeof(melody1929)/2);
  }
  else if (A == 4) {
    digitalWrite(LEFT, HIGH);
    digitalWrite(CENTER, HIGH);
    digitalWrite(RIGHT, HIGH);
    play_melody(melody4,beats4,sizeof(melody4)/2);
  }
  else if (A == 3) {
    digitalWrite(LEFT, LOW);
    digitalWrite(CENTER, LOW);
    digitalWrite(RIGHT, HIGH);
    play_melody(melody3,beats3,sizeof(melody3)/2);
  }
  else if (A == 2) {
    digitalWrite(LEFT, LOW);
    digitalWrite(CENTER, HIGH);
    digitalWrite(RIGHT, LOW);
    play_melody(melody2,beats2,sizeof(melody2)/2);
  }
  else if (A == 1) {
    digitalWrite(LEFT, HIGH);
    digitalWrite(CENTER, LOW);
    digitalWrite(RIGHT, LOW);
    play_melody(melody1,beats1,sizeof(melody1)/2);
  }
}

