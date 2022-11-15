#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
Adafruit_DCMotor *myMotorLeft = AFMS.getMotor(1);
Adafruit_DCMotor *myMotorRight= AFMS.getMotor(2);

const int leftSensor = 2; // assign pin Names for readability, prevent "what is that pin"
const int rightSensor = 3; // change pins here, or plug into these on the arduino

const int onOff = 8; // button for turning on and off
bool button = LOW;

int leftLine = 0; // initialise line "bool" values
int rightLine = 0;

int oldLeftSpeed = 0; // to avoid the arduino updating constantly
int oldRightSpeed = 0;

int baseMotorSpeed = 255; // Change these to affect the speed and turning circle
int lowTurnSpeed = 10;

void leftTurn(){
  if (oldLeftSpeed != lowTurnSpeed){ //check if speeds actually need updating
  myMotorRight->setSpeed(baseMotorSpeed); // set right motor high speed and
  oldRightSpeed = baseMotorSpeed; // update speed for update requirement check
  myMotorLeft->setSpeed(lowTurnSpeed); // left motor low speed to perform turn with large radius
  oldLeftSpeed = lowTurnSpeed;
  myMotorLeft->run(FORWARD);
  myMotorRight->run(FORWARD);
  return;
  }
}

void rightTurn(){
  if (oldRightSpeed != lowTurnSpeed){
    myMotorLeft->setSpeed(baseMotorSpeed);
    oldLeftSpeed = baseMotorSpeed;
    myMotorRight->setSpeed(lowTurnSpeed);
    oldRightSpeed = lowTurnSpeed;
    myMotorLeft->run(FORWARD);
    myMotorRight->run(FORWARD);
    return;
  }
}

void forwards(){
  if (oldRightSpeed != baseMotorSpeed or oldLeftSpeed != baseMotorSpeed){
    myMotorLeft->setSpeed(baseMotorSpeed);
    myMotorRight->setSpeed(baseMotorSpeed);
    oldLeftSpeed = baseMotorSpeed;
    oldRightSpeed = baseMotorSpeed;
    myMotorLeft->run(FORWARD);
    myMotorRight->run(FORWARD);
    return;
  }
}

void lineFollowing(){
  leftLine = digitalRead(leftSensor);
  rightLine = digitalRead(rightSensor);

  if (leftLine == HIGH){
    leftTurn();
    }
  else{ if (rightLine == HIGH ){
    rightTurn();
    }
  else{
    forwards();
    }
  }
}

void setup() {
  // put your setup code here, to run once:
AFMS.begin();
myMotorLeft->setSpeed(127); //left wheel
myMotorRight->setSpeed(127);// right wheel
oldLeftSpeed = 127;
oldRightSpeed = 127;
pinMode(leftSensor, INPUT); //leftlightsensor
pinMode(rightSensor, INPUT); //Rightlightsensor
}



void loop() {
  // put your main code here, to run repeatedly:
  lineFollowing();
}
