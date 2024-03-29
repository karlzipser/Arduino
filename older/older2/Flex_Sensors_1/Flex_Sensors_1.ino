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

char* Flexes[]={"xfl0","xfl1","xfc0","xan0","xfr0","xfr1","xbl0", "xbl1","xbr1","xbr0"};

float baselines[] = {0,0,0,0,0,0,0,0,0,0,0,0};

int flex_pins[] = {A0};//,A1,A2,A3,A4,A5,A6,A7,A8};//,A9,A10,A11};
//int flex_pins[] = {A9};
int num_pins = 1;
const float VCC = 5.0; // Estimated voltage of Ardunio 5V line
//const float R_DIV = 47500.0; // Measured resistance of 47k resistor
const float R_DIV = 10000.0; // Estimated resistance of 10k resistor

const int TAB_FORMAT = 1;



float get_flexR(int pin)
{
  int flexADC;
  float flexV;
  float flexR;
  flexADC = analogRead(flex_pins[pin]);
  flexV = flexADC / 100.0 * VCC / 1023.0;
  flexR = R_DIV / 100.0 * (VCC / flexV - 1.0);
  return flexADC;//flexR;
}

void setup() 
{
  Serial.begin(115200);
  for( int i = 0; i < num_pins; i = i + 1 ) {
        pinMode(flex_pins[i], INPUT);
    for( int j = 0; j < 100; j = j + 1 ) {
      baselines[i] += get_flexR(i);
    }
    baselines[i] /= 100.0;
  }
  
}

float running_baseline = 0;
void loop() 
{
  float bflexR;
  float s = 0.999;
  for( int i = 0; i < num_pins; i = i + 1 ) {
    bflexR = get_flexR(i);// - baselines[i];
    bflexR -= running_baseline;
    running_baseline = (1-s)*bflexR+s*running_baseline;
    if (not TAB_FORMAT) Serial.print("('");
    if (not TAB_FORMAT) Serial.print(bflexR);
    if (not TAB_FORMAT) Serial.print("',");
    Serial.print(0);Serial.print('\t');
    Serial.print(bflexR);Serial.print('\t');
    Serial.println(400);//int(running_baseline+20));
    //if (TAB_FORMAT) Serial.print('\t');
    //else Serial.println(")");
  }

  if (TAB_FORMAT) Serial.println("");
  delay(0);

}

