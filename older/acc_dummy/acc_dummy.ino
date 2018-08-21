#include <SoftwareSerial.h>
#include <Wire.h>


void setup()  
{
  
  Serial.begin(115200);
   
}



//////////////////////////////////////////////////////

float ax = 0;
float ay = 0;
float az = 0;
float ctr = 0;

uint32_t timer = millis();
void loop() {
  
 
  if (timer > millis())  timer = millis();

  if (millis() - timer > 20) {
    timer = millis();
    Serial.print("('acc',");
    Serial.print(99); Serial.print(",");
    Serial.print(99); Serial.print(",");
    Serial.print(99); Serial.print(")");
    Serial.println();
    
    Serial.print("('gyro',");
    Serial.print(9); Serial.print(",");
    Serial.print(9); Serial.print(",");
    Serial.print(9); Serial.print(")");
    Serial.println();

  }


}


