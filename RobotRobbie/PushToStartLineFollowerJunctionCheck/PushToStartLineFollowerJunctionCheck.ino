#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
Adafruit_DCMotor *myMotorLeft = AFMS.getMotor(1);
Adafruit_DCMotor *myMotorRight= AFMS.getMotor(2);

const int leftSensor1 = 1; // assign pin Names for readability
const int leftSensor2 = 2; // prevent "what is that pin" question
const int rightSensor1 = 3; // change pins here 
const int rightSensor2 = 4; // plug into these on the arduino

const int onOff = 8; // button for turning on and off
int button = LOW;
int begun = 0;

int leftLine = 0; // initialise line "present or missing" values
int leftJunction = 0;
int rightLine = 0;
int rightJunction = 0;

int oldLeftSpeed = 0; // to avoid the arduino updating constantly we check
int oldRightSpeed = 0; // if speed needs to be changed
int oldDirection = 0; // 0 for backwards, 1 for forwards

int baseMotorSpeed = 255; // Change these to affect the speed
int lowTurnSpeed = 10; // and the size of the turning circle

void begin(); // function checks if the start button has been pushed
void begin(){
  while (begun != 1){
    int button = digitalRead(onOff);
    if (button == HIGH) {
    begun = 1;
    }
  }
  return;
}

void forwards(); // both driving motors set equal and forward
void forwards(){
  if (oldDirection != 1){
    if (oldRightSpeed != baseMotorSpeed or oldLeftSpeed != baseMotorSpeed){
      myMotorLeft->setSpeed(baseMotorSpeed);
      myMotorRight->setSpeed(baseMotorSpeed);
      oldLeftSpeed = baseMotorSpeed;
      oldRightSpeed = baseMotorSpeed;
      myMotorLeft->run(FORWARD);
      myMotorRight->run(FORWARD);
      oldDirection = 1;
    }
  }
}

void backwards(); // both driving motors set equal and backward
void backwards(){
  if (oldDirection != 0){
    if (oldRightSpeed != baseMotorSpeed or oldLeftSpeed != baseMotorSpeed){
    myMotorLeft->setSpeed(baseMotorSpeed);
    myMotorRight->setSpeed(baseMotorSpeed);
    oldLeftSpeed = baseMotorSpeed;
    oldRightSpeed = baseMotorSpeed;
    myMotorLeft->run(BACKWARD);
    myMotorRight->run(BACKWARD);
    oldDirection = 0;
    }
  }
}

void rightTurn(); // right motor set low, both forwards
void rightTurn(){
  if (oldLeftSpeed != baseMotorSpeed){ //check if speeds actually need updating
    myMotorLeft->setSpeed(baseMotorSpeed);
    oldLeftSpeed = baseMotorSpeed;
    myMotorRight->setSpeed(lowTurnSpeed);
    oldRightSpeed = lowTurnSpeed;
    myMotorLeft->run(FORWARD);
    myMotorRight->run(FORWARD);
  }
}

void leftTurn(); // left motor set low, both forwards
void leftTurn(){
  if (oldRightSpeed != baseMotorSpeed){
    myMotorRight->setSpeed(baseMotorSpeed);
    oldRightSpeed = baseMotorSpeed;
    myMotorLeft->setSpeed(lowTurnSpeed);
    oldLeftSpeed = lowTurnSpeed;
    myMotorLeft->run(FORWARD);
    myMotorRight->run(FORWARD);
  }
}

void followLine(); // combines the above with line sensors to follow line
void followLine(){
  leftLine = digitalRead(leftSensor1);
  rightLine = digitalRead(rightSensor1);
  if (leftLine == HIGH){
    rightTurn();
  } else{ if (rightLine == HIGH ){
    leftTurn();
    }
  else{
    forwards();
    }
  }
  delay(100);
  button = digitalRead(onOff);
  if (button = HIGH){
    begun = 0; // if button is pushed then vehicle stops as begin loop is entered
  }
}
void checkJunction();
void checkJunction(){
  leftJunction = digitalRead(leftSensor2);
  rightJunction = digitalRead(rightSensor2);
  if (leftJunction == HIGH){
    backwards();
    delay(1000);
    leftTurn();  
  } else{ if (rightJunction == HIGH){
    backwards();
    delay(1000);
    rightTurn();
  }else{    
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
  pinMode(leftSensor1, INPUT); // innerLeftLightSensor
  pinMode(leftSensor2, INPUT); // outerLeftLightSensor
  pinMode(rightSensor1, INPUT); // innerRightLightSensor
  pinMode(rightSensor2, INPUT); // outerRightLightSensor
}

void loop() {
  // put your main code here, to run repeatedly:
  begin();
  followLine();
  checkJunction();
}
