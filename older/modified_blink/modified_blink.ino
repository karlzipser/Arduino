/*
  Blink
  Turns on an LED on for one second, then off for one second, repeatedly.

  Most Arduinos have an on-board LED you can control. On the Uno and
  Leonardo, it is attached to digital pin 13. If you're unsure what
  pin the on-board LED is connected to on your Arduino model, check
  the documentation at http://www.arduino.cc

  This example code is in the public domain.

  modified 8 May 2014
  by Scott Fitzgerald
 */


// the setup function runs once when you press reset or power the board
void setup() {
    Serial.begin(115200);
    Serial.setTimeout(5);
  // initialize digital pin 13 as an output.
  pinMode(12, OUTPUT);
  pinMode(11, OUTPUT);
  digitalWrite(11, LOW);   // turn the LED on (HIGH is the voltage level)
  digitalWrite(12, LOW);   // turn the LED on (HIGH is the voltage level)

}
int delay_time = 1000;
// the loop function runs over and over again forever
void loop() {
  Serial.println("('signals,0')");
  int signal = Serial.parseInt();
 
  if (signal == 2){
    digitalWrite(12, HIGH);
  }
  else if (signal == 1) {
    digitalWrite(12, LOW);
    }

}
