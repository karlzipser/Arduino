void setup() {
  // put your setup code here, to run once:
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

void loop() {
  // put your main code here, to run repeatedly:
  long int A = Serial.parseInt();
  if (A>0){
    long int B = A - (int(A/1000))*1000;
    long int C = A - B;
    Serial.print(A); Serial.print('\t');
    Serial.print(B); Serial.print('\t');
    Serial.println(C);
    //Serial.println("\");// Serial.print('\t');
  }
  /*
  int s_led = int(A/10000);
  int s_motor = int(A-10000*s_led);
  if (A>0){
    Serial.print(A); Serial.print('\t');
    Serial.print(A-s_led); Serial.print('\t');
    Serial.print(s_motor); Serial.print('\t');
    Serial.println(s_led);
  */
  //}
}
