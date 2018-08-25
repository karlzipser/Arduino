#include <SoftwareSerial.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>

#define TCAADDR 0x70

void tcaselect(uint8_t i) {
  if (i > 7) return;
 
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();  
}

///////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////
//////////// gyro0_scope //////////////////////////////////////////////////////////
// PINS: A4,A5
// Baud: 115200
// http://forum.arduino.cc/index.php?topic=147351.0
// http://www.livescience.com/40103-accelerometer-vs-gyro0_scope.html
//Parallax gyro0_scope Module 3-Axis L3G4200D
//https://www.parallax.com/product/27911
//http://forum.arduino.cc/index.php?topic=147351.msg1106879#msg1106879
//VIN to +5V
//GND to Ground
//SCL line to pin A5
//SDA line to pin A4
#include <Wire.h>
#define  CTRL_REG1  0x20
#define  CTRL_REG2  0x21
#define  CTRL_REG3  0x22
#define  CTRL_REG4  0x23
#define  CTRL_REG5  0x24
#define  CTRL_REG6  0x25
int gyro0_I2CAddr=70;//105;
int gyro0_Raw[3];                         // raw sensor data, each axis, pretty useless really but here it is.
double gyro0_DPS[3];                      // gyro0_ degrees per second, each axis
float heading[3]={0.0f};                // heading[x], heading[y], heading [z]
int gyro0_ZeroRate[3];                    // Calibration data.  Needed because the sensor does center at zero, but rather always reports a small amount of rotation on each axis.
int gyro0_Threshold[3];                   // Raw rate change data less than the statistically derived threshold is discarded.
#define  NUM_gyro0__SAMPLES  50           // As recommended in STMicro doc
#define  gyro0__SIGMA_MULTIPLE  3         // As recommended 
float dpsPerDigit=.00875f;              // for conversion to degrees per second
void gyro0__setup() {
  tcaselect(0);
  //Serial.begin(115200);
  Serial.println("here1");
  Wire.begin();
  Serial.println("here2");
  setupgyro0_();
  Serial.println("here3");
  calibrategyro0_();
  Serial.println("here4");
}
void gyro0__loop() {
  Serial.println("here5");
  tcaselect(0);
  Serial.println("here6");
  updategyro0_Values();
  Serial.println("here7");
  updateHeadings();
  Serial.println("here8");
  /*
  Serial.print("(");
  Serial.print(STATE_gyro0_);
  Serial.print(",");
  Serial.print(gyro0_DPS[0]);
  Serial.print(",");
  Serial.print(gyro0_DPS[1]);
  Serial.print(",");
  Serial.print(gyro0_DPS[2]);
  Serial.println(")");
  */
  //printDPS();
  //Serial.print("   -->   ");
  printHeadings3();
  //Serial.println();
}
void printDPS()
{
  Serial.print("DPS X: ");
  Serial.print(gyro0_DPS[0]);
  Serial.print("  Y: ");
  Serial.print(gyro0_DPS[1]);
  Serial.print("  Z: ");
  Serial.print(gyro0_DPS[2]);
}
void printHeadings()
{
  Serial.print("Heading X: ");
  Serial.print(heading[0]);
  Serial.print("  Y: ");
  Serial.print(heading[1]);
  Serial.print("  Z: ");
  Serial.print(heading[2]);
}
void printHeadings2()
{
  Serial.print("('head',");
  Serial.print(heading[0]);
  Serial.print(',');

  Serial.print(heading[1]);
  Serial.print(',');

  Serial.print(heading[2]);
  Serial.println(')');
}
void printHeadings3()
{
  /*
  Serial.print(heading[0]);
  Serial.print('\t');
  Serial.print(heading[1]);
  Serial.print('\t');
  */
  Serial.print(heading[2]);
  Serial.println();
}
void updateHeadings()
{
  float deltaT=getDeltaTMicros();
  for (int j=0;j<3;j++)
    heading[j] -= (gyro0_DPS[j]*deltaT)/1000000.0f;
}
unsigned long getDeltaTMicros()
{
  static unsigned long lastTime=0;
  unsigned long currentTime=micros();
  unsigned long deltaT=currentTime-lastTime;
  if (deltaT < 0.0)
     deltaT=currentTime+(4294967295-lastTime);
  lastTime=currentTime;
  return deltaT;
}
void testCalibration()
{
  calibrategyro0_();
  for (int j=0;j<3;j++)
  {
    Serial.print(gyro0_ZeroRate[j]);
    Serial.print("  ");
    Serial.print(gyro0_Threshold[j]);
    Serial.print("  ");  
  }
  Serial.println();
  return; 
}
void setupgyro0_()
{
  gyro0_WriteI2C(CTRL_REG1, 0x1F);        // Turn on all axes, disable power down
  gyro0_WriteI2C(CTRL_REG3, 0x08);        // Enable control ready signal
  setgyro0_Sensitivity500();
  delay(100);
}
void calibrategyro0_()
{
  long int gyro0_Sums[3]={0};
  long int gyro0_Sigma[3]={0};
  for (int i=0;i<NUM_gyro0__SAMPLES;i++)
  {
    updategyro0_Values();
    for (int j=0;j<3;j++)
    {
      gyro0_Sums[j]+=gyro0_Raw[j];
      gyro0_Sigma[j]+=gyro0_Raw[j]*gyro0_Raw[j];
    }
  }
  for (int j=0;j<3;j++)
  {
    int averageRate=gyro0_Sums[j]/NUM_gyro0__SAMPLES;
    gyro0_ZeroRate[j]=averageRate;
    gyro0_Threshold[j]=sqrt((double(gyro0_Sigma[j]) / NUM_gyro0__SAMPLES) - (averageRate * averageRate)) * gyro0__SIGMA_MULTIPLE;    
  }
}
void updategyro0_Values() {
  while (!(gyro0_ReadI2C(0x27) & B00001000)){}      // Without this line you will get bad data occasionally
  int reg=0x28;
  for (int j=0;j<3;j++)
  {
    gyro0_Raw[j]=(gyro0_ReadI2C(reg) | (gyro0_ReadI2C(reg+1)<<8));
    reg+=2;
  }
  int deltagyro0_[3];
  for (int j=0;j<3;j++)
  {
    deltagyro0_[j]=gyro0_Raw[j]-gyro0_ZeroRate[j];      // Use the calibration data to modify the sensor value.
    if (abs(deltagyro0_[j]) < gyro0_Threshold[j])
      deltagyro0_[j]=0;
    gyro0_DPS[j]= dpsPerDigit * deltagyro0_[j];      // Multiply the sensor value by the sensitivity factor to get degrees per second.
  }
}
void setgyro0_Sensitivity250(void)
{
  dpsPerDigit=.00875f;
  gyro0_WriteI2C(CTRL_REG4, 0x80);        // Set scale (250 deg/sec)
}
void setgyro0_Sensitivity500(void)
{
  dpsPerDigit=.0175f;
  gyro0_WriteI2C(CTRL_REG4, 0x90);        // Set scale (500 deg/sec)
}
void setgyro0_Sensitivity2000(void)
{
  dpsPerDigit=.07f;
  gyro0_WriteI2C(CTRL_REG4,0xA0); 
}
int gyro0_ReadI2C (byte regAddr) {
  Wire.beginTransmission(gyro0_I2CAddr);
  Wire.write(regAddr);
  Wire.endTransmission();
  Wire.requestFrom(gyro0_I2CAddr, 1);
  while(!Wire.available()) {};
  return (Wire.read());
}
int gyro0_WriteI2C( byte regAddr, byte val){
  Wire.beginTransmission(gyro0_I2CAddr);
  Wire.write(regAddr);
  Wire.write(val);
  Wire.endTransmission();
}
//
void setup()  
{
  tcaselect(0);

  Serial.begin(115200);
  Serial.println("setup");
  gyro0__setup();
  
}



//////////////////////////////////////////////////////

float ax = 0;
float ay = 0;
float az = 0;
float ctr = 0;

uint32_t timer = millis();
void loop() {
 
    gyro0__loop();
  


}

//////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////



