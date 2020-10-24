#include <Encoder.h>
Encoder motorRight(3,5);
Encoder motorLeft(2,6);
const int directionOutRight = 7;
const int directionOutLeft = 8;
const int speedOutRight = 9;
const int speedOutLeft = 10;
const int powerPin = 4;
//PID Controller Values
const double kP = 12.049; //Kp in units of V/deg
const double kI = 16.385; //Ki in units of V*s/deg
const double kD = 4.7196; //Kd in units of V/(deg*s)
long newTime;
double integral;
void setup() {
  Serial.begin(250000);
  Serial.println("Ready");
  pinMode(directionOutRight, OUTPUT);
  pinMode(directionOutLeft, OUTPUT);
  pinMode(speedOutRight, OUTPUT);
  pinMode(speedOutLeft, OUTPUT);
  pinMode(powerPin, OUTPUT);
  newTime = micros();
  integral = 0.0;
}
long positionLeft = 0;
long positionRight = 0;
long oneFootCounts = 7850;//1865 for ~90 degrees, ~865 for 45
long deltaT;
double oldError = 0.0;
long numSteers = 1;
boolean moved = false;
void loop() {
  steer(numSteers*oneFootCounts);
  numSteers = 0;
  moveOneFoot(moved);
  moved = true;
}
void steer(long countChange){
   while (countChange != 0){
    analogWrite(speedOutRight, 127); //default to 50% duty cycle
    analogWrite(speedOutLeft, 127);
    if (countChange > 0){ //set proper direction
      digitalWrite(directionOutRight, LOW);
      digitalWrite(directionOutLeft, HIGH);
    }
    else{
      digitalWrite(directionOutRight, HIGH);
      digitalWrite(directionOutLeft, LOW);
    }
    digitalWrite(powerPin, HIGH); //give power to the motors
    long newPosRight = motorRight.read();
    long newPosLeft = motorLeft.read();
    if (newPosLeft != positionLeft || newPosRight != positionRight)
    {
      //Serial.println(newPos);
      newTime = micros();
      double error = (double)(newPosLeft - positionLeft)*3.14/1600; //calculated error value from position counts for PID controller
      double deriv;
      if (deltaT > 0){
        deriv = (error - oldError)*1000000.0/deltaT; //derivative value to be multiplied by Kd in PID controller
        oldError = error;
      }
      else{
        deriv = 0;
      }
      integral = integral + deltaT/1000000.0*error; //integral value to be multiplied by Ki in PID controller
      int u = kP*error + kI*integral + kD*deriv; //PID controlled velocity
      if (u > 0){
        digitalWrite(directionOutRight, LOW); //change the direction if the velocity is negative or positive depending on which direction
        digitalWrite(directionOutLeft, HIGH);
      }
      else{
        digitalWrite(directionOutRight, HIGH);
        digitalWrite(directionOutLeft, LOW);
      }
      u = abs(u);
      analogWrite(speedOutRight, u);
      analogWrite(speedOutLeft, u); //set new duty cycle as PID controlled value
  
      deltaT = micros() - newTime;
      newTime = micros();
      //testing block
      if (newPosLeft == (positionLeft + countChange) || newPosRight == (positionRight + countChange)){ //stop condition
        //Serial.println(newPos);
        if (countChange > 0){
          digitalWrite(directionOutRight, HIGH);
          digitalWrite(directionOutLeft, LOW);
        }
        else{
          digitalWrite(directionOutRight, LOW);
          digitalWrite(directionOutLeft, HIGH);
        }
        countChange = 0;
        delay(100);
        digitalWrite(powerPin, LOW);
        positionLeft = newPosLeft;
        positionRight = newPosRight;
      }
    }
  }
}

void moveOneFoot(boolean moved){
  long countChange = 1975;
  if (moved){
    countChange = 0;
  }
  while (countChange != 0){
    analogWrite(speedOutRight, 127); //default to 50% duty cycle
    analogWrite(speedOutLeft, 127);
    if (countChange > 0){ //set proper direction
      digitalWrite(directionOutRight, HIGH);
      digitalWrite(directionOutLeft, HIGH);
    }
    else{
      digitalWrite(directionOutRight, LOW);
      digitalWrite(directionOutLeft, LOW);
    }
    digitalWrite(powerPin, HIGH); //give power to the motors
    long newPosRight = motorRight.read();
    long newPosLeft = motorLeft.read();
    if (newPosLeft != positionLeft || newPosRight != positionRight)
    {
      //Serial.println(newPos);
      newTime = micros();
      double error = (double)(newPosLeft - positionLeft)*3.14/1600; //calculated error value from position counts for PID controller
      double deriv;
      if (deltaT > 0){
        deriv = (error - oldError)*1000000.0/deltaT; //derivative value to be multiplied by Kd in PID controller
        oldError = error;
      }
      else{
        deriv = 0;
      }
      integral = integral + deltaT/1000000.0*error; //integral value to be multiplied by Ki in PID controller
      int u = kP*error + kI*integral + kD*deriv; //PID controlled velocity
      if (u > 0){
        digitalWrite(directionOutRight, HIGH); //change the direction if the velocity is negative or positive depending on which direction
        digitalWrite(directionOutLeft, HIGH);
      }
      else{
        digitalWrite(directionOutRight, LOW);
        digitalWrite(directionOutLeft, LOW);
      }
      u = abs(u);
      analogWrite(speedOutRight, u);
      analogWrite(speedOutLeft, u); //set new duty cycle as PID controlled value
  
      deltaT = micros() - newTime;
      newTime = micros();
      //testing block
      if (newPosLeft == (positionLeft + countChange) || newPosRight == (positionRight + countChange)){ //stop condition
        //Serial.println(newPos);
        if (countChange > 0){
          digitalWrite(directionOutRight, LOW);
          digitalWrite(directionOutLeft, LOW);
        }
        else{
          digitalWrite(directionOutRight, HIGH);
          digitalWrite(directionOutLeft, HIGH);
        }
        countChange = 0;
        delay(100);
        digitalWrite(powerPin, LOW);
        positionLeft = newPosLeft;
        positionRight = newPosRight;
      }
    }
  }
}
