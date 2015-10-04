/*
This code receives an encoded Netstring message and 
decodes it acording to Netsring protocols. Then the 
decoded data is transferred to two servos, one to steer 
the car and another to move the car at a given speed. 
Author: Neda Ashrafi Amiri
*/
#include <Servo.h>
Servo esc; //initialize esc type Servo
Servo servo; //initialize servo type Servo
String c; //initialize encoded Netsrting received via Serial COM
String decodedS; // initialize decoded String
int throttle = 0;
int steer=60; 
int control =512;

void setup() {
  esc.attach(9);//attach esc to pin 9 on Mega
  servo.attach(10);// attach servo to pin 10 Mega
  Serial.begin(115200);
}

void loop() {
  throttle = map(control, 0, 1023, 1000, 2000); 
  Serial.println(control);
  esc.write(throttle); 
  servo.write(steer);
  delay(10);
}

void serialEvent() {
  if(Serial.available()) {
    c = Serial.readStringUntil(',');
    decodedS=decodedString(c);
    delay(2);  //slow looping to allow buffer to fill with next character


    if (decodedS.length() >0) {
     // Serial.println(decodedS);  //so you can see the captured string 
      control = decodedS.substring(0, 3).toInt();  //convert a substring readString into a number
      steer= decodedS.substring(3, 6).toInt(); //convert a substring readString into a number
    }
    decodedS="";
    c="";
  }
}



String decodedString(String string){
  //if string is less than 8 chars, it's either invalid or empty string
  //e.g. 512030, ---> this is alltogether 8 characters containing /0
  if (string.length() < 6) return "error"; 
  // we always have a checksum of 6 by default
  int controlDigits = 6;
  String command = string; 
  // if it's an empty string, return "error"
  if (!command.length()) return "error"; 
   //if string's length isn't 
   //equal with the control digits, it's an invalid Netstring
  if (command.length() != controlDigits) return "error"; 
  return command;
}
