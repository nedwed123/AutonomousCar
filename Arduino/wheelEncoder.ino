/*
This code reads tick signals from the wheel encoder Plolulu 
using interrupts via Outport A only (NOt using poth outPOrts 
of the wheel encoder )

Author: Neda Ashrafi Amiri
*/
volatile unsigned long ticks = 0; 

void setup()
{
  Serial.begin(9600);
   
  attachInterrupt(5, encoderInterrupt, RISING); // int 5 represents pin 18 on mega
}

void loop()
{
  Serial.println(ticks);
  delay(100);
}

// Interrupt service routine for the wheel encoder
void encoderInterrupt()
{
  ticks++;
}
