#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
Adafruit_DCMotor *myMotor1 = AFMS.getMotor(1);
Adafruit_DCMotor *myMotor2= AFMS.getMotor(2);
int leftsensor = 0; 
int rightsensor = 0;

void setup() {
  // put your setup code here, to run once:
AFMS.begin();
myMotor1->setSpeed(127); //left wheel
myMotor2->setSpeed(127);// right wheel
pinMode(3, INPUT); //leftlightsensor
pinMode(4, OUTPUT); 
pinMode(5, INPUT); //Rightlightsensor
pinMode(6, OUTPUT);
}



void loop() {
  // put your main code here, to run repeatedly:
  leftsensor = digitalRead(3);
  rightsensor = digitalRead(5);


if (leftsensor == HIGH){

  myMotor1->setSpeed(255);
  myMotor2->setSpeed(10);
  myMotor1->run(BACKWARD);
  myMotor2->run(BACKWARD);

} else{ if (rightsensor == HIGH ){
  myMotor1->setSpeed(10);
  myMotor2->setSpeed(255);
  myMotor1->run(BACKWARD);
  myMotor2->run(BACKWARD);
}else{
  myMotor1->setSpeed(255);
  myMotor2->setSpeed(255);
  myMotor1->run(BACKWARD);
  myMotor2->run(BACKWARD);

}


}

}