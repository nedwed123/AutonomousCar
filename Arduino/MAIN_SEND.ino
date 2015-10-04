/*
This code receievs data from three infrareds, 2 ultrasonics 
of type srf08, one ultrasonic of type sr04 and a pololu wheel encoder.
The received data is then encoded into Netstring messages and sent through
Serial COM. 

Author: Neda Ashrafi Amiri
*/

#include <Wire.h>

#define srfAddress1 0x70    // Address of the SRF08
#define srfAddress2 0x73    // Address of the SRF08

#define cmdByte 0x00        // Command byte
#define lightByte 0x01      // Byte to read light sensor
#define rangeByte 0x02      // Byte for start of ranging data

volatile unsigned long ticks = 0; //encoder tick intialized

const int trigPinM = 42;    //Ultrosonic SR04 trigger
const int echoPinM= 43;     //Ultrosonic SR04 echo

const int irPin1 = 15;      //Infrared defined pin
const int irPin2 = 14;      //Infrared defined pin
const int irPin3 = 13;      //Infrared defined pin

long ir1, ir2, ir3;         //initialize infrared sensor value
long us1, us2, us3;         //initialize ultrasonic sensor value

String input;               //String of sensor inputs 
String encodedS;            //String of sensor inputs 
                              //after being encoded by Netstrings
String inf1, inf2, inf3;    //initialize infrared String 
                              //(later we transform long to String)

String ult1, ult2, ult3;    //initialize ultrasonic String
                              //(later we transform long to String)
String encoder;

void setup() {
  Wire.begin();             // initialize the Wire object     
  pinMode(irPin1,OUTPUT);   // setup infrared pin as OUTPUT
  pinMode(irPin2,OUTPUT);   // setup infrared pin as OUTPUT
  pinMode(irPin3,OUTPUT);   // setup infrared pin as OUTPUT
  Serial.begin(9600);       // initialize serial communication:
  attachInterrupt(5, encoderInterrupt, CHANGE);//5 represents pin 18 on Mega
}

void loop()
{
  ir1 = infraredSense(irPin1);  // assign infraredsense function return to ir1
  ir2 = infraredSense(irPin2);  // assign infraredsense function return to ir2
  ir3 = infraredSense(irPin3);  // assign infraredsense function return to ir3

  us1= ultrasonicSense(trigPinM, echoPinM);  //assign ultrasonicsense function return to us1
  us2= getRange(srfAddress1);   // assign getrange function return to us2
  us3= getRange(srfAddress2);   // assign getrange function return to us3

  inf1=checkLength(String(ir1));
  inf2=checkLength(String(ir2));
  inf3=checkLength(String(ir3));
  ult1=checkLength(String(us1));
  ult2=checkLength(String(us2));
  ult3=checkLength(String(us3));
  encoder= String(ticks);

  input+= inf1+ inf2+ inf3 + ult1+ ult2+ ult3+ encoder;
  encodedS=encodedNetstring(input);

  Serial.println(encodedS);
  encodedS="";
  input="";
  delay(100);
}


// This function encodes a String as a netstring
String encodedNetstring(String plainInput){
  unsigned int len = plainInput.length(); 
  if (!len) return "error"; //if the input string is empty, return "error"
  return len + String(":" + plainInput + ","); //return a Netstring in the form of [len]":"[string]","
}
long infraredSense(const int pin){
  // A variable to store the values from sensor.
  // This is initially zero.
  int val;

  // A variable to store the calculated cm value
  int cm;

  // read the value of the sharp sensor on A3
  val = analogRead(pin);

  // Apply Linearisation
  cm = (2914 / (val +5))-1;

  // prints the value of the sensor to serial
  // Serial.print("Infrared cm: ");
  //Serial.print(cm);
  //Serial.println();

  return cm;
}

long ultrasonicSense(int trigPin, int echoPin){
  // establish variables for duration of the ping, 
  // and the distance result in inches and centimeters:
  long duration,  cm;

  // The sensor is triggered by a HIGH pulse of 10 or more microseconds.
  // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
  pinMode(trigPin, OUTPUT);
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Read the signal from the sensor: a HIGH pulse whose
  // duration is the time (in microseconds) from the sending
  // of the ping to the reception of its echo off of an object.
  pinMode(echoPin, INPUT);
  duration = pulseIn(echoPin, HIGH);

  // convert the time into a distance
  cm = microsecondsToCentimeters(duration);

  //Serial.print(cm);
  //Serial.print("cm for US1");
  //Serial.println();
  return cm;
}

long microsecondsToCentimeters(long microseconds)
{
  // The speed of sound is 340 m/s or 29 microseconds per centimeter.
  // The ping travels out and back, so to find the distance of the
  // object we take half of the distance travelled.
  return microseconds / 29 / 2;
} 

// This function gets a ranging from the SRF08
int getRange(int srfAddress){                       
  int range = 0; 
  Wire.beginTransmission(srfAddress);             // Start communicating with SRF08
  Wire.write((byte)cmdByte);                      // Send Command Byte
  Wire.write(0x51);                               // Send 0x51 to start a ranging in cm
  Wire.endTransmission();

  delay(100);                                     // Wait for ranging to be complete

  Wire.beginTransmission(srfAddress);             // start communicating with SRFmodule
  Wire.write(rangeByte);                          // Call the register for start of ranging data
  Wire.endTransmission();

  Wire.requestFrom(srfAddress, 2);                // Request 2 bytes from SRF module
  while(Wire.available() < 2);                    // Wait for data to arrive
  byte highByte = Wire.read();                    // Get high byte
  byte lowByte = Wire.read();                     // Get low byte

  range = (highByte << 8) + lowByte;              // Put them together

  return(range);                                  // Returns Range
}


//  This function checks the length of a sensor value (or any other string)
//  and adds 0 or 00 accordingly to make the length of the string precisely 3
String checkLength(String value){
    //  then if the length of the string is 2 it adds a 0 to the beginning of that string
    if(value.length()==2){
      value= "0"+ value;
    }
     //  then if the length is 1 it adds two 00s to the beginning of that String
    else if(value.length()==1){
      value="00"+ value;
    }
    //  then if the length is 3 it returns the value itself.
    else if(value.length()==3){
    value = value;
  }
  //  all other cases it returns the value itself. 
  return value;
}
  
// Interrupt service routine for the wheel encoder
void encoderInterrupt()
{
  ticks++;   //increment tick
}
