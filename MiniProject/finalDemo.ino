#include <Encoder.h> //encoder library
#include <Wire.h> //library to allow communication between the arduino and raspberry pi

//Pin assignments
#define SLAVE_ADDRESS 0x04
Encoder motor(3,5);
const int directionOut = 7;
const int speedOut = 9;
const int powerPin = 4;
//PID Controller Values
const double kP = 6.049; //Kp in units of V/deg
const double kI = 10.385; //Ki in units of V*s/deg
const double kD = 3.7196; //Kd in units of V/(deg*s)
int currentAngle = 4; //current angle relative to the quadrant of the ARUCO
//0 = 4
//pi/2 = 3
//pi = 2
// 3pi/2 = 1
int countChange = 0; //this variable tracks distance and direction the wheel will have to travel to get to its destination
long newTime;
long deltaT = 0;
double integral;
double oldError = 0.0;
int sent = 0;
void setup() {
  Serial.begin(250000);
  Wire.begin(SLAVE_ADDRESS); //address to connect to PI
  Wire.onReceive(receiveData);
  Wire.onRequest(sendData);
  Serial.println("Ready");
  pinMode(directionOut, OUTPUT);
  pinMode(speedOut, OUTPUT);
  pinMode(powerPin, OUTPUT);
}
long position = 0;
void loop() 
{
  countChange = 0;
  int byteCount = 1;
  receiveData(byteCount);
  //case block for determining where the wheel has to go
  if (currentAngle == 4){
    if (sent == 3){
      countChange = 800;
      currentAngle = sent;
    }
    else if (sent == 2){
      countChange = 1600;
      currentAngle = sent;
    }
    else if (sent == 1){
      countChange = -800;
      currentAngle = sent;
    }
    else{
      countChange = 0;
    }
  }
  else if (currentAngle == 3){
    if (sent == 4){
      countChange = -800;
      currentAngle = sent;
    }
    else if (sent == 2){
      countChange = 800;
      currentAngle = sent;
    }
    else if (sent == 1){
      countChange = 1600;
      currentAngle = sent;
    }
    else{
      countChange = 0;
    }
  }
  else if (currentAngle == 2){
    if (sent == 4){
      countChange = 1600;
      currentAngle = sent;
    }
    else if (sent == 3){
      countChange = -800;
      currentAngle = sent;
    }
    else if (sent == 1){
      countChange = 800;
      currentAngle = sent;
    }
    else{
      countChange = 0;
    }
  }
  else if (currentAngle == 1){
    if (sent == 4){
      countChange = 800;
      currentAngle = sent;
    }
    else if (sent == 3){
      countChange = 1600;
      currentAngle = sent;
    }
    else if (sent == 2){
      countChange = -800;
      currentAngle = sent;
    }
    else{
      countChange = 0;
    }
  }
  //loop to run motor
  while (countChange != 0){
    analogWrite(speedOut, 127); //default to 50% duty cycle
    if (countChange > 0){ //set proper direction
      digitalWrite(directionOut, LOW);
    }
    else{
      digitalWrite(directionOut, HIGH);
    }
    digitalWrite(powerPin, HIGH); //give power to the motor
    long newPos = motor.read();
    if (newPos != position)
    {
      //Serial.println(newPos);
      newTime = micros();
      double error = (double)(newPos - position)*3.14/1600; //calculated error value from position counts for PID controller
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
        digitalWrite(directionOut, LOW); //change the direction if the velocity is negative or positive depending on which direction
      }
      else{
        digitalWrite(directionOut, HIGH);
      }
      u = abs(u);
      analogWrite(speedOut, u); //set new duty cycle as PID controlled value
  
      deltaT = micros() - newTime;
      newTime = micros();
      //testing block
      if (newPos == (position + countChange)){ //stop condition
        Serial.println(newPos);
        if (countChange > 0){
          digitalWrite(directionOut, HIGH);
        }
        else{
          digitalWrite(directionOut, LOW);
        }
        countChange = 0;
        sendData(); //send data to be displayed on LCD
        sent = 0;
        delay(100);
        digitalWrite(powerPin, LOW);
        position = newPos;
      }
    }
  }
}

void receiveData(int byteCount)
{
  while(Wire.available())
  {
    sent = Wire.read(); //loop to read more than one byte sent from the PI into an array
  }
}
void sendData()
{
  int sendtoPI = 0;
 int converted = position%3200; //modulus added to help determine quadrant for 1 rotation
 if(converted <= 400 && converted >= 2800)
 {
  sendtoPI = 4;
 }
 else if(converted <= 1200 && converted > 400)
 {
  sendtoPI = 3;
 }
 else if(converted <= 2000 && converted > 1200)
 {
  sendtoPI = 2;
 }
 else if(converted <= 2800 && converted > 2000)
 {
  sendtoPI = 1;
 }
 Wire.write(sendtoPI);
}
