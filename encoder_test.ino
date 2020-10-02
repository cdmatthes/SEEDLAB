#include <Encoder.h>
Encoder knobLeft(3,5);
const int powerPin = 4;
const int directionOut = 7;
const int speedOut = 9;
double newTime;
double oldTime;
double deltaT;
double velocity;
void setup() {
  Serial.begin(9600);
  Serial.println("Encoder Test:");
  pinMode(powerPin, OUTPUT);
  digitalWrite(powerPin, HIGH);
  pinMode(directionOut, OUTPUT);//Kp V/deg; Kd V*s/deg; Ki V/(deg*s)
  digitalWrite(directionOut, LOW);
  pinMode(speedOut, OUTPUT);
}
long positionLeft = -999;
double startTime = 0;
void loop() {
  //analogWrite(speedOut, 127);
  if (millis() == 1000){
    digitalWrite(speedOut, HIGH);
    startTime = micros();
  }
  if (millis() >= 6000){
    digitalWrite(speedOut, LOW);
  }
  long newLeft;
  newLeft = knobLeft.read();
  if (newLeft != positionLeft){
    newTime = (double)micros();
    
//    Serial.print("Left = ");
//    Serial.print(newLeft);
    Serial.print("\t");
    
    deltaT = newTime - oldTime;
    velocity = (double)(newLeft - positionLeft) / deltaT;
    velocity = velocity*3.14*1000000.0/1600.0;
    Serial.print(velocity);
    Serial.print("\t");
    Serial.print(newTime - startTime);
    Serial.print("\n");

    oldTime = newTime;
    positionLeft = newLeft;
  }
  if (Serial.available()){
    Serial.read();
    Serial.println("Knob reset to zero");
    knobLeft.write(0);
  }
  //delay(millis()%50);
}
