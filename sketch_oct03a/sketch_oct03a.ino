
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
int CENTER = 3;
int RIGHT = 4;
int BAG = 5;
int ZED = 6;
int LIDAR = 7;

void setup()
{
  pinMode(1, OUTPUT);digitalWrite(1, LOW);
  pinMode(2, OUTPUT);digitalWrite(2, LOW);
  pinMode(3, OUTPUT);digitalWrite(3, LOW);
  pinMode(4, OUTPUT);digitalWrite(4, LOW);
  pinMode(5, OUTPUT);digitalWrite(5, LOW);
  pinMode(6, OUTPUT);digitalWrite(6, LOW);
  pinMode(7, OUTPUT);digitalWrite(7, LOW);
  pinMode(8, OUTPUT);digitalWrite(8, LOW);
  pinMode(9, OUTPUT);digitalWrite(9, LOW);
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



int light_on = 1;

int high;

void loop() {

  int A;
  A = Serial.parseInt();
  digitalWrite(1, LOW);
  digitalWrite(2, LOW);
  digitalWrite(3, LOW);
  digitalWrite(4, LOW);
  digitalWrite(5, LOW);
  digitalWrite(6, LOW);
  digitalWrite(7, LOW);
  digitalWrite(8, LOW);
  digitalWrite(9, LOW);
  digitalWrite(A,HIGH);
  delay(500);

  
}



