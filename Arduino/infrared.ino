/*
This code reads data from an infrared sensor GP2D120 
and transforms the reading into Centimiters.
AUthor: Neda Ashrafi Amiri
*/
const int irPin1 = 15;
long ir1;

void setup() {
  // Set the pinmode to output
  pinMode(irPin1,OUTPUT);

  
  Serial.begin(9600);
  
}
void loop(){
  
ir1 = infraredSense(irPin1);
  delay(100);

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
  Serial.print("cm: ");
  Serial.print(cm);
  Serial.println();
  
  return cm;
}
