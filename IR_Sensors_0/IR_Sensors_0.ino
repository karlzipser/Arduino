/* d *****************************************************************************

Based on:
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


char* Flexes[]={"FR0","FR1", "FL2","FL3", "FC0", "FL0","FL1", "FL2","FR3"};

//FL2 'sparks', FR3 'sparks'

float baselines[] = {0,0,0,0,0,0,0,0,0,0,0,0};

int flex_pins[] = {A0,A1, A2,A3, A4, A6,A7};//, A10,A11};

int num_pins = 1;
const float VCC = 5.0; // Estimated voltage of Ardunio 5V line
//const float R_DIV = 47500.0; // Measured resistance of 47k resistor
const float R_DIV = 10000.0; // Estimated resistance of 10k resistor

const int TAB_FORMAT = 1;



float get_IR(int pin)
{
  int flexADC;
  flexADC = analogRead(flex_pins[pin]);
  return flexADC;
}

void setup() 
{
  Serial.begin(115200);
}

float IR_prev = 0;
float s = 0.95;
long unsigned int now = millis();
void loop() 
{
  
  
  for( int i = 0; i < num_pins; i = i + 1 ) {
    float IR = get_IR(i); // - baselines[i];
    IR = s*IR_prev + (1.0-s)*IR;
    IR_prev = IR; 
    if (millis()>now+50) {
      Serial.print(IR);
      if (TAB_FORMAT) Serial.print('\t');
      else Serial.println(")");
      now = millis();
    }
  }

  if (TAB_FORMAT) Serial.println("");

}

