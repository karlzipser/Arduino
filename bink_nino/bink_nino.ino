
void setup() {
  pinMode(13, OUTPUT);
}


void loop() {

  int a = 60;
  int b = 1000/a;
  digitalWrite(13, 1);
  delay(b);                      
  digitalWrite(13, 0);
  delay(b);                 
}
