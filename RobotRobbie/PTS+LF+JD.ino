#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
Adafruit_DCMotor *myMotorLeft = AFMS.getMotor(1);
Adafruit_DCMotor *myMotorRight= AFMS.getMotor(2);

const int leftLineSensorOuter = 1;
const int leftLineSensorInner = 2;  
const int rightLineSensorInner = 3;
const int rightLineSensorOuter = 4;
// assign pin Names for readability, prevent "what is that pin"
// change pins here, or plug into these on the arduino

const int onOff = 8; // button for turning on and off
int button = 0;
int begun = 0;

int leftJunction = 0; // initialise line "bool" values
int leftLine = 0;
int rightLine = 0;
int rightJunction = 0;


int oldLeftSpeed = 0; // to avoid the arduino updating constantly
int oldRightSpeed = 0;
int oldLeftDir = 0; // 0 for backwards, 1 for forwards
int oldRightDir = 0;

const int baseMotorSpeed = 255; // Change these to affect the speed and turning circle
const int lowTurnSpeed = 10;

void leftTurn(){
  if (oldLeftSpeed != lowTurnSpeed){ //check if speeds actually need updating
  myMotorRight->setSpeed(baseMotorSpeed); // set right motor high speed and
  oldRightSpeed = baseMotorSpeed; // update speed for update requirement check
  myMotorLeft->setSpeed(lowTurnSpeed); // left motor low speed to perform turn with large radius
  oldLeftSpeed = lowTurnSpeed;
  myMotorLeft->run(FORWARD);
  myMotorRight->run(FORWARD);
  oldLeftDir = 1;
  oldRightDir = 1;
  return;
  }
}

void sharpLeft(){
  if (oldLeftDir != 0){ //check if speeds actually need updating
    myMotorRight->setSpeed(baseMotorSpeed); // set right motor high speed and
    oldRightSpeed = baseMotorSpeed; // update speed for update requirement check
    myMotorLeft->setSpeed(baseMotorSpeed); // left motor low speed to perform turn with large radius
    oldLeftSpeed = baseMotorSpeed;
    myMotorLeft->run(BACKWARD);
    myMotorRight->run(FORWARD);
    oldLeftDir = 0;
    oldRightDir = 1;
  return;
  }
}

void sharpRight(){
  if (oldRightDir != 0){ //check if speeds actually need updating
    myMotorRight->setSpeed(baseMotorSpeed); // set right motor high speed and
    oldRightSpeed = baseMotorSpeed; // update speed for update requirement check
    myMotorLeft->setSpeed(baseMotorSpeed); // left motor low speed to perform turn with large radius
    oldLeftSpeed = baseMotorSpeed;
    myMotorLeft->run(FORWARD);
    myMotorRight->run(BACKWARD);
    oldLeftDir = 1;
    oldRightDir = 0;
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
    oldLeftDir = 1;
    oldRightDir = 1;
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
    oldLeftDir = 1;
    oldRightDir = 1;
    return;
  }
}

void stop(){
  if (oldRightSpeed != 0 or oldLeftSpeed != 0){
    myMotorLeft->setSpeed(0);
    myMotorRight->setSpeed(0);
    oldLeftSpeed = 0;
    oldRightSpeed = 0;
    myMotorLeft->run(FORWARD);
    myMotorRight->run(FORWARD);
    oldLeftDir = 1;
    oldRightDir = 1;
    return;
  }
}

void lineFollowing(){
  leftLine = digitalRead(leftLineSensorInner);
  rightLine = digitalRead(rightLineSensorInner);

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
  return;
}

void begin(){
  if (begun != 1){  
    while (true){
      button = digitalRead(onOff);
      if (button == 1){
        begun = 1;
        delay(500); // prevent button immediately being read again for the check to stop
        break;
      }
    }
  }
  return;
}

void finish(){
  int count = 0;
  while (count < 100){ // prevents getting stuck in the finish check
    button = digitalRead(onOff);
    if (button == 1){
      begun = 0;
      stop();
      delay(500);
      break;
    }
    count += 1;
  }
  return;  
}

void setup() {
  // put your setup code here, to run once:
// motor setup
AFMS.begin();
myMotorLeft->setSpeed(127); //left wheel
myMotorRight->setSpeed(127);// right wheel
oldLeftSpeed = 127;
oldRightSpeed = 127;

// button setup
pinMode(onOff, INPUT); // push to make button

// line sensors setup
pinMode(leftLineSensorOuter, INPUT);
pinMode(leftLineSensorInner, INPUT); // inner left line sensor
pinMode(rightLineSensorInner, INPUT); // inner right line sensor
pinMode(rightLineSensorOuter, INPUT);
}



void loop() {
  // put your main code here, to run repeatedly:
  begin();
  lineFollowing();
  leftJunction = digitalRead(leftLineSensorOuter);
  rightJunction = digitalRead(rightLineSensorOuter);
  if (leftJunction == HIGH){
    sharpLeft();
    delay(1000);
    while (true){
      rightLine = digitalRead(rightLineSensorInner);
      if (rightLine == HIGH){
        break;
      }
    }
    sharpRight();
    delay(1000);
    while (true){
      leftLine = digitalRead(leftLineSensorInner);
      if (leftLine == HIGH){
        break;
      }
    }
    forwards();
    delay(500);
  }
  



  finish(); // need to check if should finish 
}
