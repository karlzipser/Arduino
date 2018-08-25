#include <SoftwareSerial.h>
#include <Wire.h>
#include <Adafruit_MMA8451.h>
#include <Adafruit_Sensor.h>


Adafruit_MMA8451 mma = Adafruit_MMA8451();


void setup()  
{
  Serial.begin(115200);

  Serial.println("Adafruit MMA8451 test!");
  if (! mma.begin()) {
    Serial.println("Couldnt start");
    while (1);
  }
  //Serial.println("MMA8451 found!");
  mma.setRange(MMA8451_RANGE_2_G);
  Serial.print("Range = "); Serial.print(2 << mma.getRange());  
  Serial.println("G");
}



//////////////////////////////////////////////////////


uint32_t timer = millis();
float acc_xs = 0;float acc_ys = 0;float acc_zs = 0;
void loop() {

 
  if (timer > millis())  timer = millis();

  if (millis() - timer > 1000) { 
    timer = millis();
  }  

  sensors_event_t event; 
  mma.getEvent(&event);
  acc_xs = 0.9*acc_xs + 0.1*event.acceleration.x;
  acc_ys = 0.9*acc_ys + 0.1*event.acceleration.y;
  acc_zs = 0.9*acc_zs + 0.1*event.acceleration.z;
  //Serial.print("(acc,");
  //Serial.print(event.acceleration.x); Serial.print("\t");
  Serial.print(acc_xs); Serial.print("\t");
  Serial.print(acc_ys); Serial.print("\t");
  Serial.print(acc_zs); Serial.print("\t");
  //Serial.print(event.acceleration.y); Serial.print("\t");
  //Serial.println(event.acceleration.z); //Serial.print(")");
  Serial.println();
  delay(1000/100);

}
