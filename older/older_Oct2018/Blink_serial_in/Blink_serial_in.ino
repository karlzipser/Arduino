
void setup() {

  for (int i=0;i<14;++i){
    pinMode(i, OUTPUT);digitalWrite(i, LOW);
  }
  
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
   Serial.println("Enter 0-13 for led pin, or 20-3000 for delay");
}

int d = 1000;
int p = 13;

void loop() {

  int A = Serial.parseInt();
  if (A>=20) {
    if (A<=300) {
      d = A;
      Serial.print("entered ");
      Serial.print(d);
      Serial.println(" as delay.");
    }
  } else if (A>0){
    if (A<14){
      p = A;
      Serial.print("entered ");
      Serial.print(p);
      Serial.println(" as pin number.");
    }
  } 
  digitalWrite(p, HIGH);
  delay(d);
  digitalWrite(p, LOW);
  delay(d);      
  

}
