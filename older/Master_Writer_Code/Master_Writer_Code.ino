// Wire Master Writer
// by Nicholas Zambetti <http://www.zambetti.com>

// Demonstrates use of the Wire library
// Writes data to an I2C/TWI slave device
// Refer to the "Wire Slave Receiver" example for use with this

// Created 29 March 2006

// This example code is in the public domain.


#include <Wire.h>

void setup() {
  Serial.begin(115200);
  Wire.begin(); // join i2c bus (address optional for master)
}

byte x = 0;

int an_int;
void loop() {
  int an_int = Serial.parseInt();
  //Serial.println(an_int);
  
  byte a_byte;
  if (an_int <= 256) {
    a_byte = an_int - 1;
    Wire.beginTransmission(8); // transmit to device #8
    Wire.write(a_byte);              // sends one byte
    Wire.endTransmission();    // stop transmitting
    an_int += 1;
    Serial.println(an_int);
 
  }

  delay(0);
}
