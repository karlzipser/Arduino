
/////////////////////// LIGHTS ///////////////////////////////
//

int RIGHT_YELLOW = 9;
int LEFT_YELLOW = 12;
int BLUE = 5;
int WHITE = 7;
int PURPLE = 118;
int GREEN = 6;
int LEFT_RED = 11;
int RIGHT_RED = 10;
int LEFT_GREEN = 13;
int RIGHT_GREEN = 8;
int LIGHTS_OUT = 22;
int LIGHTS_ENABLED = 21;
int HUMAN_DRIVER = 100;
int NOT_HUMAN_DRIVER = 101;
int WHITE_OUT = 117;
int GREEN_OUT = 116;
int BLUE_OUT = 115;
int PURPLE_OUT = 119;
int LEFT_GREEN_OFF = 121;
int RIGHT_GREEN_OFF = 123;
int PURPLE_PIN = 4;
//
//////////////////////////////////////////////////////

int button;
int lights_out = 0;
int human_driver = 1;


void begin_serial()
{
  Serial.begin(115200);
  Serial.setTimeout(5);
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

void setup()
{
    for (int i=4; i<14;++i){
      pinMode(i, OUTPUT);digitalWrite(i, LOW);
    }
    begin_serial();
    Serial.println("lights_and_tones__led_master.ino setup()");    
}

int toggle(int t) {
  if (t==0) t=1;
  else t = 0;
  return t;
}

int ___tic___ = 1;

long unsigned int ___prev_time___ = millis();

int tic_toc(int delay_time)
{
  if (delay_time < 0) return 1;
  long unsigned int now = millis();
  if (now - ___prev_time___ > delay_time){
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

int red_delay = -1;

void loop() {
  
  int A;

  if (millis()-now_sound > 5000) {
    now_sound = millis();
    Serial.println("('sound',0,0,0)");
  }

  A = Serial.parseInt();

  int level = tic_toc(250);

  
  
  int red_level = 1;
  /*
  if (A >=-100&&A<-90) red_delay = 10*(-A-90);
  int red_level = tic_toc(red_delay);
  Serial.println(red_delay);
  Serial.println(red_level);
  */
  if (A == LIGHTS_OUT) {
    lights_out = 1;
  }
  else if (A == LIGHTS_ENABLED) {
    lights_out = 0;
  }

  if (lights_out == 1) {
    for (int i = 0; i< 14;++i){
        pinMode(i,OUTPUT);digitalWrite(i,LOW);
    }
    delay(100);
    return;
  }

  if (A == HUMAN_DRIVER) {
    human_driver = 1;
  }
  else if (A == NOT_HUMAN_DRIVER) {
    human_driver = 0;
  }
  else if (A == BLUE) {
    digitalWrite(BLUE, HIGH);
  }
  else if (A == BLUE_OUT) {
    digitalWrite(BLUE, LOW);
  }
  else if (A == PURPLE) {
    digitalWrite(PURPLE_PIN, HIGH);
  }
  else if (A == PURPLE_OUT) {
    digitalWrite(PURPLE_PIN, LOW);
  }
  else if (A == WHITE) {
    digitalWrite(WHITE, HIGH);
  }
  else if (A == WHITE_OUT) {
    digitalWrite(WHITE, LOW);
  }
  else if (A == GREEN) {
    digitalWrite(GREEN, HIGH);
  }
  else if (A == GREEN_OUT) {
    digitalWrite(GREEN, LOW);
  }
  else if (A == LEFT_GREEN) {
    digitalWrite(LEFT_GREEN, HIGH);
  }
  else if (A == LEFT_GREEN_OFF) {
    digitalWrite(LEFT_GREEN, LOW);
  }
  else if (A == RIGHT_GREEN) {
    digitalWrite(RIGHT_GREEN, HIGH);
  }
  else if (A == RIGHT_GREEN_OFF) {
    digitalWrite(RIGHT_GREEN, LOW);
  }
  else if (A < 5 && A > 0) { 
    button = A;
  }

  if (button == 2) {      
      digitalWrite(LEFT_RED, red_level);
      digitalWrite(RIGHT_RED, HIGH);
      digitalWrite(LEFT_YELLOW, LOW);
      digitalWrite(RIGHT_YELLOW, LOW);
  }

  

  if (button == 1) {
    digitalWrite(LEFT_YELLOW, level);
    digitalWrite(RIGHT_RED, HIGH);
    digitalWrite(RIGHT_YELLOW, LOW);
    digitalWrite(LEFT_RED, level);
  }
  if (button == 3) {
    digitalWrite(RIGHT_YELLOW, level);
    digitalWrite(LEFT_RED, HIGH);
    digitalWrite(LEFT_YELLOW, LOW);
    digitalWrite(RIGHT_RED, level);
  }   

  if (button == 4) {
    digitalWrite(LEFT_YELLOW, level);
    digitalWrite(RIGHT_YELLOW, level);
    digitalWrite(LEFT_RED, LOW);
    digitalWrite(RIGHT_RED, LOW);
  }
   
  
}

//EOF
