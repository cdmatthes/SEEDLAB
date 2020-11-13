#include <Wire.h> 
#define SLAVE_ADDRESS 0x04
signed int receive[3]; //will only need distance and angle
int distance = 0;
int angle = 999;
char sendOut[100];
int str2 = 0;
int fin = 0;
int len = 0;
int a = 0;
String st;
#include <Encoder.h> //define encoders
Encoder motorRight(3,5);
Encoder motorLeft(2,6);
const int directionOutRight = 7;
const int directionOutLeft = 8;
const int speedOutRight = 9;
const int speedOutLeft = 10;
const int powerPin = 4;
//PID Controller Values
const double kP = 12.049; //Kp in units of V/deg
const double kI = 10.385; //Ki in units of V*s/deg
const double kD = 8.7196; //Kd in units of V/(deg*s)
const double kPInnerCircle = 6.049; //Kp value for inner wheel when circling beacon
long oneFootCounts = 1975;
long newTime;
double integral;
double integralInner; //separate PID terms for circle
double integralOuter;
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

  pinMode(13, OUTPUT);
  
  Wire.begin(SLAVE_ADDRESS); //address to connect to PI

  Wire.onReceive(receiveData); //calls function to receive data from RPI

  Serial.println("Ready");
}
long deltaT;
double oldError = 0.0;
const long oneEightyDegreeCounts = 3200;//3200/180
long positionLeft = 0;
long positionRight = 0;
double oldErrorInner = 0.0; //multiple PID error values for cricle
double oldErrorOuter = 0.0;
boolean hasRun = false;
void loop() {
  // put your main code here, to run repeatedly:
  if (hasRun == false){
    searchForBeacon();
    moveMotorForward(distance);
    integral = 0.0;
//    delay(400); //this code is commented out for when the robot only wants to approach the beacon not circle it
//    steer(90);
//    integralInner = 0.0;
//    integralOuter = 0.0;
//    circleBeacon();
    hasRun = true;
  }
}

void receiveData(int byteCount)
{
  a = 0;
  while(Wire.available())
  {
    receive[a] = Wire.read();
   a++;
  }
  distance = receive[1];
  angle = receive[2];
  if (angle > 90)
  {
    angle = angle - 256;
  }
   Serial.println(distance);
  Serial.println(angle);
  
  }

void searchForBeacon(){
  while (angle > 60 || angle < -60){ //while the marker is out of the camera's field of vision
     long countChange = oneEightyDegreeCounts*45/180;
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
    delay(2000); //give the camera a few seconds to detect the aruco marker
  }
  
  long countChange = oneEightyDegreeCounts*(-1*angle)/180;
  integral = 0.0;
  
  while (countChange != 0){ //angle adjusting loop
    analogWrite(speedOutRight, 32); //start at small duty cycle since most adjustments will be small
    analogWrite(speedOutLeft, 32);
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
        digitalWrite(powerPin, LOW);
        positionLeft = newPosLeft;
        positionRight = newPosRight;
      }
    }
  }
}

void moveMotorForward(long numInches){
  //loop to run motor
  long countChange;
  countChange = numInches*oneFootCounts/12 - 7*oneFootCounts/8; //subtract 7/8 of a foot to ensure not crashing into beacon
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

void steer(long degree){
  motorLeft.write(0); //reset encoder counts to guarantee no issue in steering due to overshoot
  motorRight.write(0);
  positionLeft = 0;
  positionRight = 0;
  long countChange = oneEightyDegreeCounts*degree/180;
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
      if (newPosLeft == (positionLeft + countChange) || newPosRight == (positionRight - countChange)){ //stop condition
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
        delay(50);
        digitalWrite(powerPin, LOW);
        positionLeft = newPosLeft;
        positionRight = newPosRight;
      }
    }
  }
}

void circleBeacon(){
  //loop to run motor
  double innerCircleDistance = (5.8/12.0)*2*3.14*4/3; //calculate circle distances
  double outerCircleDistance = innerCircleDistance + 2*3.14*(13.0/12.0);
  long innerCircleCounts = innerCircleDistance*1975*4/3;
  long outerCircleCounts = outerCircleDistance*2170*4/3;
  while (innerCircleCounts != 0 && outerCircleCounts != 0){
    analogWrite(speedOutRight, 127); //default to 50% duty cycle for outer wheel
    analogWrite(speedOutLeft, 40); //default to just above 25% duty cycle for inner wheel
    //difference in duty cycle determines size of the radii
    if (innerCircleCounts > 0){ //set proper direction
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
      double errorInner = (double)(newPosLeft - positionLeft)*3.14/1600; //calculated error value from position counts for PID controller
      double errorOuter = (double)(newPosRight - positionRight)*3.14/1600;
      double derivInner;
      double derivOuter;
      if (deltaT > 0){
        derivInner = (errorInner - oldErrorInner)*1000000.0/deltaT; //derivative value to be multiplied by Kd in PID controller
        derivOuter = (errorOuter - oldErrorOuter)*1000000.0/deltaT;
        oldErrorInner = errorInner;
        oldErrorOuter = errorOuter;
      }
      else{
        derivInner = 0;
        derivOuter = 0;
      }
      integralInner = integralInner + deltaT/1000000.0*errorInner; //integral value to be multiplied by Ki in PID controller
      integralOuter = integralOuter + deltaT/1000000.0*errorOuter;
      int uInner = kPInnerCircle*errorInner + kI*integralInner + kD*derivInner; //PID controlled velocity
      int uOuter = kP*errorOuter + kI*integralOuter + kD*derivOuter;
      if (uInner > 0){
        //change the direction if the velocity is negative or positive depending on which direction
        digitalWrite(directionOutLeft, HIGH);
      }
      else{
        digitalWrite(directionOutLeft, LOW);
      }
      if (uOuter > 0){
        digitalWrite(directionOutRight, HIGH);
      }
      else{
        digitalWrite(directionOutRight, LOW);
      }
      uInner = abs(uInner);
      uOuter = abs(uOuter);
      analogWrite(speedOutRight, uOuter);
      analogWrite(speedOutLeft, uInner); //set new duty cycle as PID controlled value
  
      deltaT = micros() - newTime;
      newTime = micros();
      //testing block
      if (newPosRight == (positionRight + outerCircleCounts)){
        if (outerCircleCounts > 0){
          digitalWrite(directionOutRight, LOW);
        }
        else{
          digitalWrite(directionOutRight, HIGH);
        }
        outerCircleCounts = 0;
        delay(100);
        analogWrite(speedOutRight, 0);
        positionRight = newPosRight;
      }
      if (newPosLeft == (positionLeft + innerCircleCounts)){ //stop condition
        //Serial.println(newPos);
        if (innerCircleCounts > 0){
          digitalWrite(directionOutLeft, LOW);
        }
        else{
          digitalWrite(directionOutLeft, HIGH);
        }
        innerCircleCounts = 0;
        delay(100);
        analogWrite(speedOutLeft, 0);
        positionLeft = newPosLeft;
      }
    }
  }
  digitalWrite(powerPin, LOW);
}
