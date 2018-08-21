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
const int FLEX_PIN0 = A1; // Pin connected to voltage divider output
const int FLEX_PIN1 = A0; // Pin connected to voltage divider output

// Measure the voltage at 5V and the actual resistance of your
// 47k resistor, and enter them below:
const float VCC = 4.98; // Measured voltage of Ardunio 5V line
const float R_DIV = 47500.0; // Measured resistance of 3.3k resistor

// Upload the code, then try to adjust these values to more
// accurately calculate bend degree.
const float STRAIGHT_RESISTANCE = 37300.0; // resistance when straight
const float BEND_RESISTANCE = 90000.0; // resistance at 90 deg

void setup() 
{
  Serial.begin(115200);
  pinMode(FLEX_PIN0, INPUT);
  pinMode(FLEX_PIN1, INPUT);
}

void loop() 
{
  // Read the ADC, and calculate voltage and resistance from it
  int flexADC0 = analogRead(FLEX_PIN0);
  float flexV = flexADC0 * VCC / 1023.0;
  float flexR = R_DIV * (VCC / flexV - 1.0);
  //Serial.println("Resistance: " + String(flexR) + " ohms");
  //Serial.println(FLEX_PIN0);
  //Serial.println(FLEX_PIN1);
  Serial.print(flexR);Serial.print('\t');
  
  float flexADC1 = analogRead(FLEX_PIN1);
  flexV = flexADC1 * VCC / 1023.0;
  flexR = R_DIV * (VCC / flexV - 1.0);
  //Serial.println("Resistance: " + String(flexR) + " ohms");
  Serial.println(String(flexR));
  
  // Use the calculated resistance to estimate the sensor's
  // bend angle:
  float angle = map(flexR, STRAIGHT_RESISTANCE, BEND_RESISTANCE,
                   0, 90.0);
  //Serial.println("Bend: " + String(angle) + " degrees");
  //Serial.println();

  delay(10);
}

