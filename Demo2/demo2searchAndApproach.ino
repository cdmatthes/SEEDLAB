#include <Wire.h> 
#define SLAVE_ADDRESS 0x04
signed int receive[3];
int distance = 0;
int angle = 999;
char sendOut[100];
int str2 = 0;
int fin = 0;
int len = 0;
int a = 0;
String st;
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
const double kI = 12.385; //Ki in units of V*s/deg
const double kD = 5.7196; //Kd in units of V/(deg*s)
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
boolean hasRun = false;
void loop() {
  // put your main code here, to run repeatedly:
  if (hasRun == false){
    searchForBeacon();
    moveOneFoot();
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
  int byteCount = 2;
  while (angle > 90 || angle < -90){
    //receiveData(byteCount);
    Serial.println(angle);
    analogWrite(speedOutRight, 32);
    analogWrite(speedOutLeft, 32);
    digitalWrite(directionOutRight, LOW);
    digitalWrite(directionOutLeft, HIGH);
    digitalWrite(powerPin, HIGH);
    if (angle < 90 && angle > -90){
      digitalWrite(directionOutRight, HIGH);
      digitalWrite(directionOutLeft, LOW);
      delay(100);
      digitalWrite(powerPin, LOW);
      positionLeft = motorLeft.read();
      positionRight = motorRight.read();
    }
  }
  long countChange = oneEightyDegreeCounts*(angle - 15)/180;
  
  while (countChange != 0){
    analogWrite(speedOutRight, 32); //default to 50% duty cycle
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

void moveOneFoot(){
  long countChange = 1975;
  while (countChange != 0){
    analogWrite(speedOutRight, 127); //default to 50% duty cycle
    analogWrite(speedOutLeft, 127);
    digitalWrite(directionOutRight, HIGH);
    digitalWrite(directionOutLeft, HIGH);
    
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
        digitalWrite(directionOutRight, LOW);
        digitalWrite(directionOutLeft, LOW);
        countChange = 0;
        delay(100);
        digitalWrite(powerPin, LOW);
        positionLeft = newPosLeft;
        positionRight = newPosRight;
      }
    }
  }
}
