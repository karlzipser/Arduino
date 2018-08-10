/******************************************************************************
Flex_Sensor_Example.ino
Example sketch for SparkFun's flex sensors
  (https://www.sparkfun.com/products/10264)
Jim Lindblom @ SparkFun Electronics
April 28, 2016

Create a voltage divider circuit combining a flex sensor with a 47k resistor.
- The resistor should connect from A0 to GND.
- The flex sensor should connect from A0 to 3.3V
As the resistance of the flex sensor increases (meaning it's being bent), the
voltage at A0 should decrease.

Development environment specifics:
Arduino 1.6.7
******************************************************************************/

//int flex_pins[] = {A11,A0,A1,A2,A3,A4,A5,A6,A7,A8,A9,A10,A11};
int flex_pins[] = {A6,A7,A8,A9,A10,A11};
int num_pins = 6;
const float VCC = 5.0; // Estimated voltage of Ardunio 5V line
//const float R_DIV = 47500.0; // Measured resistance of 47k resistor
const float R_DIV = 10000.0; // Estimated resistance of 10k resistor

void setup() 
{
  Serial.begin(115200);
  for( int i = 0; i < num_pins; i = i + 1 ) {
        pinMode(flex_pins[i], INPUT);;
     }
}

int flexADC;
float flexV;
float flexR;

void loop() 
{
  for( int i = 0; i < num_pins; i = i + 1 ) {
    flexADC = analogRead(flex_pins[i]);
    flexV = flexADC * VCC / 1023.0;
    flexR = R_DIV * (VCC / flexV - 1.0);
    Serial.print(flexR);Serial.print('\t');
  }
  Serial.println("");
  delay(10);
}

