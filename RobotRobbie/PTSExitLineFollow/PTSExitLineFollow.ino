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

void stop(){
  if (oldRightSpeed != 0 or oldLeftSpeed != 0){
    myMotorLeft->setSpeed(0);
    myMotorRight->setSpeed(0);
    oldLeftSpeed = 0;
    oldRightSpeed = 0;
    myMotorLeft->run(FORWARD);
    myMotorRight->run(FORWARD);
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
pinMode(leftLineSensorInner, INPUT); // inner left light sensor
pinMode(rightLineSensorInner, INPUT); //Rightlightsensor
while (true){
  button = digitalRead(onOff); // initial start before start/stop in main loop
  if (button == HIGH){
    begun = 1;
    break;    
  }
}
forwards(); // exit the box
delay(1000); // adjust delay numbers when mass or speed of robot changes
rightTurn(); // current values work well
delay(2500); 
stop();
}



void loop() {
  // put your main code here, to run repeatedly:
  begin();
  lineFollowing();
  finish(); // need to check if should finish 
}
