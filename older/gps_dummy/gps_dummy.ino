#include <SoftwareSerial.h>
#include <Wire.h>


void setup()  
{
  
  Serial.begin(115200);
   
}



//////////////////////////////////////////////////////


uint32_t timer = millis();
void loop() {
  
 
  if (timer > millis())  timer = millis();

  if (millis() - timer > 1000) {
    timer = millis();
    Serial.print("('gps',");
    Serial.print(99); Serial.print(",");
    Serial.print(99); Serial.print(")");
    Serial.println();
 
    Serial.print("('GPS2',");
    Serial.print(99); Serial.print(",");
    Serial.print(99); Serial.print(",");
    Serial.print(99); Serial.print(")");
    Serial.println();
 
  }


}


