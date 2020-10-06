#include <Encoder.h>
Encoder motor(3,5);
const int directionOut = 7;
const int speedOut = 9;
const int powerPin = 4;
const double kP = 12.049; //Kp in units of V/deg
const double kI = 16.385; //Ki in units of V*s/deg
const double kD = 0.7196; //Kd in units of V/(deg*s)
long newTime;
long deltaT = 0;
double integral;
int oldError = 0;
void setup() {
  Serial.begin(250000);
  Serial.println("Ready");
  pinMode(directionOut, OUTPUT);
  digitalWrite(directionOut, HIGH);
  pinMode(speedOut, OUTPUT);
  pinMode(powerPin, OUTPUT);
  digitalWrite(powerPin, HIGH);
}
long position = 0;
void loop() {
  analogWrite(speedOut, 127);
  long newPos = motor.read();
  if (newPos != position){
    //Serial.println(newPos);
    newTime = micros();
    int error = newPos - position;
    double deriv;
    if (deltaT > 0){
      deriv = (error - oldError)*1000000.0/deltaT;
      oldError = error;
    }
    else{
      deriv = 0;
    }
    integral = integral + deltaT/1000000.0*error;
    int u = kP*error + kI*integral + kD*deriv;
    analogWrite(speedOut, u);

    deltaT = micros() - newTime;
    newTime = micros();
    //testing block
//    if (newPos == -3200){
//      digitalWrite(directionOut, LOW);
//      delay(45);
//      digitalWrite(powerPin, LOW);
//    }
//    if (newPos == 3200){
//      digitalWrite(directionOut, HIGH);
//    }
  }
}
