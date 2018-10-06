

#include <Servo.h> 

Servo servo1;



void setup() 
{ 

  Serial.begin(115200);

  servo1.attach(9);
} 

void loop() 
{ 

  for (int i=60; i < 120; i++){

    servo1.write(i);
    Serial.println(i);    
    delay(50);
  }

  for (int i=1000; i < 2000; i++){

    servo1.writeMicroseconds(i);
    Serial.println(i);    
    delay(50);
  }

} 

