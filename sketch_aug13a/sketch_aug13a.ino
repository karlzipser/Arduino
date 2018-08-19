/*  Servo Recorder
This software has no warranty either real or implied.  Use at
your own risk.

Based in part on the servo and smoothing tutorials on www.arduino.cc

Records and plays back servo positions using a servo
modified with feedback.

The feedback connects to analog pin 5

The following commands are supported:
a records the current servo position
b releases the servo to record the position
c plays back the recorded position
d calibrates the end points of the servo

Every servo should be calibrated when first started to get more accurate
end points.

The minimum and maximum pulse width will vary with different servo models.  If the calibration routine does not work correctly then look up the minimum and maximum pulse widths for your servo and use them in the servo.attach lines like so:
servo.attach(SERVO_PIN,minimumPulseWidth,maximumPulseWidth);
*/

#include <Servo.h>
#define SERVO_PIN 9//5
Servo servo;

int angle = 90;
int position = 90;
int feedbackPin = 5;
int val = 0;
int calVal[] = {191, 1011};  // initial cal values
int calStartPos = 0;
int final = 90;

#define NUMREADINGS 10      // Number of readings to take for smoothing


int readings[NUMREADINGS];                // the readings from the analog input
int index = 0;                            // the index of the current reading
int total = 0;                            // the running total
int average = 0;                          // the average

void setup()
{
  Serial.begin(9600);
  //analogReference(EXTERNAL);   // Use the external analog reference  
}

void loop()
{
 static int v = 0;
  position = analogRead(feedbackPin);
  //Serial.println(position);
  position = smooth(position);

  if (1){// Serial.available()) {
    //char ch = Serial.read();
    char ch = 'e';
    switch(ch) {
      case '0'...'9':
        v = v * 10 + ch - '0';
        break;
        
      case 'a':   //  record the position of the pot  
        servo.detach();    
        Serial.print("Recording position: ");
        Serial.println(position);      
        final = position;  
        v = 0;
        break;
        
      case 'b':  //  stop playing
        Serial.println("Stop");
        servo.detach();    
        v = 0;
        break;
        
      case 'c':   // play            
        angle = map(final, calVal[0], calVal[1], 0, 180);
        Serial.print("playing: ");
        Serial.println(angle);        
        servo.attach(SERVO_PIN);  
        servo.write(angle);  
        v = 0;
        break;
        
        
      case 'd' :  // calibrate      
        Serial.println("calibrating");
        servo.attach(SERVO_PIN);
        servo.write(1);
        delay(1000);  // wait 1 second for servo to reach the position                
        calVal[0] = analogRead(feedbackPin);  
        servo.write(180);
        delay(1000);
        calVal[1] = analogRead(feedbackPin);
        Serial.print("Cal values: ");
        Serial.print(calVal[0]);
        Serial.print(",");
        Serial.println(calVal[1]);
        v = 0 ;
        break;
        
      case 'e' :  //       
        Serial.println("testing");
        servo.attach(SERVO_PIN);
        int excursion = 30;
        int null = 113;
        for( int i = null-excursion; i < null; i = i + 1 ) {
          servo.write(null);
          delay(150);  // wait 1 second for servo to reach the position                
          Serial.print(null);
          Serial.print("\t");
          Serial.println(analogRead(feedbackPin));
        }
        for( int i = null-excursion; i < null; i = i + 1 ) {
          servo.write(i);
          delay(150);  // wait 1 second for servo to reach the position                
          Serial.print(i);
          Serial.print("\t");
          Serial.println(analogRead(feedbackPin));
        }
        for( int i = null-excursion; i < null; i = i + 1 ) {
          servo.write(null);
          delay(150);  // wait 1 second for servo to reach the position                
          Serial.print(null);
          Serial.print("\t");
          Serial.println(analogRead(feedbackPin));
        }
        int i;
        for( i = null; i < null+excursion; i = i + 1 ) {
          servo.write(i);
          delay(150);  // wait 1 second for servo to reach the position                
          Serial.print(i);
          Serial.print("\t");
          Serial.println(analogRead(feedbackPin));
        }
        for( int j = null-excursion; j < null; j = j + 1 ) {
          servo.write(i);
          delay(150);  // wait 1 second for servo to reach the position                
          Serial.print(i);
          Serial.print("\t");
          Serial.println(analogRead(feedbackPin));
        } 
        break;    
    }
  }
}

int smooth(int data) {
    total -= readings[index];               // subtract the last reading
    readings[index] = analogRead(feedbackPin); // read from the sensor
    total += readings[index];               // add the reading to the total
    index = (index + 1);                    // advance to the next index

    if (index >= NUMREADINGS)               // if we're at the end of the array...
    index = 0;                            // ...wrap around to the beginning

    val = total / NUMREADINGS;          // calculate the average
    return val;
}

