/*
Generic example for the SRF modules  08.
Only the SRF08 uses the light sensor. 
*/
 
#include <Wire.h>
 
#define srfAddress1 0x70                          // Address of the SRF08
#define srfAddress2 0x73                          // Address of the SRF08

#define cmdByte 0x00                              // Command byte
#define rangeByte 0x02                            // Byte for start of ranging data
 
void setup(){
  Wire.begin();                                   // Initialize the I2C bus
  Serial.begin(9600);                             // Initialize the serial connection
  delay(100);                                     // Waits to make sure everything is powered up before sending or receiving data
}
 
void loop(){
  int rangeData1 = getRange(srfAddress1);         // Calls a function to get the range data
  int rangeData2 = getRange(srfAddress2);         // Calls a function to get the range data
 
  Serial.print("Range1: ");
  Serial.print(rangeData1);
  Serial.println("cm");
 
  Serial.print("Range2: ");
  Serial.print(rangeData2);
  Serial.println("cm");
 
  delay(100);                                      // Wait before looping
}
 
int getRange(int srfAddress){                      // This function gets a ranging from the SRF08
  int range = 0; 
 
  Wire.beginTransmission(srfAddress);             // Start communicating with SRF08
  Wire.write((byte)cmdByte);                             // Send Command Byte
  Wire.write(0x51);                                // Send 0x51 to start a ranging in cm
  Wire.endTransmission();
 
  delay(100);                                     // Wait for ranging to be complete
 
  Wire.beginTransmission(srfAddress);             // start communicating with SRFmodule
  Wire.write(rangeByte);                           // Call the register for start of ranging data
  Wire.endTransmission();
 
  Wire.requestFrom(srfAddress, 2);                // Request 2 bytes from SRF module
  while(Wire.available() < 2);                    // Wait for data to arrive
  byte highByte = Wire.read();                    // Get high byte
  byte lowByte = Wire.read();                     // Get low byte
  range = (highByte << 8) + lowByte;              // Put them together
  return(range);                                  // Returns Range
}
 

