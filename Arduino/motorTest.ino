// The Arduino code used to test the ESC and Motors
// Note not original author
#include <Servo.h>
Servo esc;
String inputString;
int throttle = 0; 
int control =512;

void setup() {
  esc.attach(9);
  Serial.begin(9600);  
}

void loop() {
  // read the value from the sensor:
  throttle = map(control, 0, 1023, 1000, 2000); 
  // display value and send to servo
  Serial.println(throttle);
  esc.writeMicroseconds(throttle); 
  delay(10);
}

void serialEvent() {
  while (Serial.available()) {
     while (Serial.available()) {
    char c = Serial.read();  //gets one byte from serial buffer
    inputString += c; //makes the string readString
    delay(2);  //slow looping to allow buffer to fill with next character
  }

  if (inputString.length() >0) {
    Serial.println(inputString);  //so you can see the captured string 
    control = inputString.toInt();  //convert readString into a number      
  }
  inputString="";
  }
}
