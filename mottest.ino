#include <Encoder.h>
#include <Wire.h>

#define SLAVE_ADDRESS 0x04
Encoder motor(3,5);
const int directionOut = 7;
const int speedOut = 9;
const int powerPin = 4;
const double kP = 12.049; //Kp in units of V/deg
const double kI = 16.385; //Ki in units of V*s/deg
const double kD = 0.7196; //Kd in units of V/(deg*s)
int currentAngle = 4; //current angle
//0 = 4
//pi/2 = 3
//pi = 2
// 3pi/2 = 1
int countChange = 0;
long newTime;
long deltaT = 0;
double integral;
int oldError = 0;
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
  while (countChange != 0){
    analogWrite(speedOut, 127);
    if (countChange > 0){
      digitalWrite(directionOut, LOW);
    }
    else{
      digitalWrite(directionOut, HIGH);
    }
    digitalWrite(powerPin, HIGH);
    long newPos = motor.read();
    if (newPos != position)
    {
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
      if (newPos == (position + countChange)){
        Serial.println(newPos);
        if (countChange > 0){
          digitalWrite(directionOut, HIGH);
        }
        else{
          digitalWrite(directionOut, LOW);
        }
        countChange = 0;
        sendData();
        sent = 0;
        Serial.println(currentAngle);
        delay(45);
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
 int converted = position%3200;
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
