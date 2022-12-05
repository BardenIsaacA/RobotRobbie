// get all packages needed for the final robot
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
#include <Arduino_LSM6DS3.h> // onboard IMU
#include <Servo.h>

// Motors and servo
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
Adafruit_DCMotor *myMotorLeft = AFMS.getMotor(1);
Adafruit_DCMotor *myMotorRight= AFMS.getMotor(2);
Servo myservo;

// Assigning names to pins for easier reading of the code later on

// IR
# define IRPin A0 // analogue input pin used by the sensor
// button for turning on and off
const int onOff = 0; // attach pin 0 Arduino to button output
// Line Sensor
const int leftLineSensorOuter = 1; // attach pin 1 Arduino to leftmost line sensor
const int leftLineSensorInner = 2; // attach pin 2 Arduino to left inner line sensor
const int rightLineSensorInner = 3; // attach pin 3 Arduino to right inner line sensor
const int rightLineSensorOuter = 4; // attach pin 4 Arduino to rightmost line sensor
// Ultrasonic
const int echoPinRight = 5; // attach pin 5 Arduino to Echo pin of right US sensor
const int trigPinRight = 6; // attach pin 6 Arduino to Trig pin of right US sensor
const int echoPinFront = 7; // attach pin 7 Arduino to Echo pin of front US sensor
const int trigPinFront = 8; // attach pin 8 Arduino to Trig pin of front US sensor
// Servo
const int servoPin = 9; // attach servo pins connected to pin 9 on the Arduino to the servo
// LEDs
const int blinkLED = 11; // amber
const int highDensityLED = 12; // red
const int lowDensityLED = 13; // green

// variables to store the button pushed state and if the program has begun (used to check if we need to enter certain functions see begin() and finish())
int button = 0;
int begun = 0;

// variables to store if there is or isnt a line detected by each sensor. 1 means a line is detected, 0 means no line
int leftOuter = 0;
int leftInner = 0;
int rightInner = 0;
int rightOuter = 0;

// variables to store junction information
int junctionCount = 0; // provides a method for the robot of "knowing" where it is in terms of junctions passed
int junctionNeeded = 3; // tells the robot which box to turn into based on which junction it just left. Starts with 3 as collecting the middle block first

// variables to store motor information to avoid the motors being unnecessarily updated
int oldLeftSpeed = 0;
int oldRightSpeed = 0;
int oldLeftDir = 0; // 0 for backwards, 1 for forwards
int oldRightDir = 0;

// values related to turning functions - righTurn, leftTurn, sharpRight, leftRight
const int baseMotorSpeed = 255; // Change these to affect the speed and turning circle
const int slowTurnSpeed = 10;
const int sharpTurnSpeed = 180;
const int USnavSpeed = 140;
const int ninetyTurnDelay = 1500;

// variables used for ultrasonic distance sensors
long duration = 0; // variable for the duration of sound wave travel
int distance = 0; // variable for the distance measurement. The same can be used for front and back as the two are never read at the same time

// values used for ultrasonic navigation of the tunnel and ramp
const int rampDist = 12;
const int tunnelDist = 8;

// variable used for IR distance sensor
int sensorValue = 0;

// variables for block based logic
int blockDensity = 0;
int hasBlock = 0;
int blocksCollected = 0;

// values for servo control
const int openPos = 180;
const int closedPos = 0; 
int currentPos = -1; // variable included to avoid running servo when unnecessary

// blink variables
int ledState = LOW;
unsigned long previousMillis = 0;
const long interval = 500;

void blink(){
  /* performs our blinking function during motion. To ensure it only occurs during motion
  we include a check that the speed is not 0 (motor speeds never negative). We used the
  inbuilt millis() function to record times, and if 500ms (f = 2Hz) has elapsed the light's
  state is changed. This is based on the example at https://www.arduino.cc/en/Tutorial/BuiltInExamples/BlinkWithoutDelay */
  if (oldLeftSpeed > 0 || oldRightSpeed > 0){
    unsigned long currentMillis = millis();

    if (currentMillis - previousMillis >= interval) {
      // save the last time you blinked the LED
      previousMillis = currentMillis;

      // if the LED is off turn it on and vice-versa:
      if (ledState == LOW) {
        ledState = HIGH;
      } else {
        ledState = LOW;
      }

      // set the LED with the ledState of the variable:
      digitalWrite(blinkLED, ledState);
    }
  }
  return;
}

// motor control functions
void leftTurn(){
  /* navigation turns are conducted by turning the inner motor slower
  than the outer (for that turn), and in the same direction */
  blink(); // we put our blink function here to ensure it runs during movement
  if (oldLeftSpeed != slowTurnSpeed){ // check if speeds or directions actually need updating, check unique to this turn
  myMotorRight->setSpeed(baseMotorSpeed); // set right motor high speed and
  oldRightSpeed = baseMotorSpeed; // update speed for update requirement check
  myMotorLeft->setSpeed(slowTurnSpeed); // left motor low speed, forwards to perform turn with large radius
  oldLeftSpeed = slowTurnSpeed;
  myMotorLeft->run(FORWARD);
  myMotorRight->run(FORWARD);
  oldLeftDir = 1; // also update motor directions for back/forward function checks
  oldRightDir = 1;
  return;
  }
}

void sharpLeft(){
  /* sharp turns are conducted by running the outer motor forwards
  and the inner motor backwards (for that turn) to produce a small
  turning cirlce */
  blink(); // we put our blink function here to ensure it runs during movement
  if (oldLeftDir != 0 or oldRightDir != 1){ // check if speeds or directions actually need updating, check unique to this turn
    myMotorRight->setSpeed(baseMotorSpeed); // set right motor high speed, forward and
    oldRightSpeed = baseMotorSpeed; // update speed for update requirement check
    myMotorLeft->setSpeed(sharpTurnSpeed); // left motor highish speed, backwards to perform turn with small radius
    oldLeftSpeed = sharpTurnSpeed; // assign this as a variable
    myMotorLeft->run(BACKWARD); // left motor runs backwards to perform sharp turn
    myMotorRight->run(FORWARD);
    oldLeftDir = 0; // also update motor directions for back/forward function checks
    oldRightDir = 1;
  return;
  }
}

void rightTurn(){
  /* navigation turns are conducted by turning the inner motor slower
  than the outer (for that turn), and in the same direction */
  blink(); // we put our blink function here to ensure it runs during movement
  if (oldRightSpeed != slowTurnSpeed){ // check if speeds or directions actually need updating, check unique to this turn
    myMotorLeft->setSpeed(baseMotorSpeed); // similar to leftTurn() but with motor speeds swapped, both forwards
    oldLeftSpeed = baseMotorSpeed;
    myMotorRight->setSpeed(slowTurnSpeed);
    oldRightSpeed = slowTurnSpeed;
    myMotorLeft->run(FORWARD);
    myMotorRight->run(FORWARD);
    oldLeftDir = 1; // also update motor directions for back/forward function checks
    oldRightDir = 1;
    return;
  }
}

void sharpRight(){
  /* sharp turns are conducted by running the outer motor forwards
  and the inner motor backwards (for that turn) to produce a small
  turning cirlce */
  blink(); // we put our blink function here to ensure it runs during movement
  if (oldRightDir != 0 or oldLeftDir != 1){ // check if speeds or directions actually need updating, check unique to this turn
    myMotorRight->setSpeed(sharpTurnSpeed); // set right motor highish speed, backward and
    oldRightSpeed = sharpTurnSpeed; // update speed for update requirement check
    myMotorLeft->setSpeed(baseMotorSpeed); // left motor high speed, forward to perform turn with small radius
    oldLeftSpeed = baseMotorSpeed;
    myMotorLeft->run(FORWARD);
    myMotorRight->run(BACKWARD); // right motor runs backwards to perform sharp turn
    oldLeftDir = 1; // also update motor directions for back/forward function checks
    oldRightDir = 0;
  return;
  }
}

void forwards(){
  /* both motors run forwards at the same speed to produce roughly straight
  forwards motion, slight motor disparities, table surface and initial error
  can cause the motor to veer off course*/
  blink(); // we put our blink function here to ensure it runs during movement
  if (oldRightSpeed != baseMotorSpeed or oldLeftSpeed != baseMotorSpeed or oldRightDir != 1 or oldLeftDir != 1){ // check if speeds or directions actually need updating, check unique to this turn
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

void backwards(){
  /* both motors run backwards at the same speed to produce roughly straight
  backwards motion, slight motor disparities, table surface and initial error
  can cause the motor to veer off course*/
  blink(); // we put our blink function here to ensure it runs during movement
  if (oldRightSpeed != baseMotorSpeed or oldLeftSpeed != baseMotorSpeed or oldLeftDir != 0 or oldRightDir != 0){
    myMotorLeft->setSpeed(baseMotorSpeed); // same as for forwards, but directions reversed
    myMotorRight->setSpeed(baseMotorSpeed);
    oldLeftSpeed = baseMotorSpeed;
    oldRightSpeed = baseMotorSpeed;
    myMotorLeft->run(BACKWARD);
    myMotorRight->run(BACKWARD);
    oldLeftDir = 0;
    oldRightDir = 0;
    return;
  }
}

void stop(){
  /* both motor speeds set to 0 to halt motion*/
  if (oldRightSpeed != 0 or oldLeftSpeed != 0){ // only need to stop if motor currently moving i.e. check if speeds or directions actually need updating, check unique to this turn
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

// distance sensor reading functions
int readIR(){
  /* analogRead(IRPin) changed to readIR() for greater
  readability*/
  sensorValue = analogRead(IRPin);
  return sensorValue;
}

int readUSRight(){
  /* reads the right ultrasonic sensor by outputting an ultrasonic
  signal, and recording the time it takes to return, then applying
  a small calculation */
  // Clears the trigPin condition
  digitalWrite(trigPinRight, LOW);
  delayMicroseconds(10);
  // Sets the trigPin HIGH (ACTIVE) for 10 microseconds
  digitalWrite(trigPinRight, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPinRight, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echoPinRight, HIGH);
  // Calculating the distance
  distance = duration * 0.034 / 2; // Speed of sound wave divided by 2 (go and back)  0.034cm/us
  return distance;
}

int readUSFront(){
  /* reads the front ultrasonic sensor by outputting an ultrasonic
  signal, and recording the time it takes to return, then applying
  a small calculation */
  // Clears the trigPin condition
  digitalWrite(trigPinFront, LOW);
  delayMicroseconds(10);
  // Sets the trigPin HIGH (ACTIVE) for 10 microseconds
  digitalWrite(trigPinFront, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPinFront, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echoPinFront, HIGH);
  // Calculating the distance
  distance = duration * 0.034 / 2; // Speed of sound wave divided by 2 (go and back)  0.034cm/us
  return distance;
}

// start stop control functions
void begin(){
  /* enters an infinite loop, provided the program is not 
  already running (begun = 1) until the button is pushed,
  when the loop is exited, and the program continues */
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
  /* we enter the finish check every run of
  the main loop as we wouldnt reach it if we weren't
  running code. tempcount is used to prevent getting
  stuck in our finish check. If the button is pushed
  the program is halted until we push the button again*/
  int tempcount = 0;
  while (tempcount < 100){ // prevents getting stuck in the finish check
    button = digitalRead(onOff);
    if (button == 1){
      begun = 0;
      stop();
      delay(500);
      break;
    }
    tempcount += 1;
  }
  return;  
}

// navigation functions
void lineFollowing(){
  /* making use of the inner line sensors and the large
  turn radius functions already written we can check if
  A: the left inner sensor sees the line, and turn left
  B: the right inner sensor sees the line, and turn right
  C: neither see the line, and go forwards */
  leftInner = digitalRead(leftLineSensorInner); // need new readings from these sensors every
  rightInner = digitalRead(rightLineSensorInner); // time we run the loop

  if (leftInner == 1){
    leftTurn();
    }
  else if (rightInner == 1){
    rightTurn();
    }
  else{
    forwards();
    }
  return;
}

void USNavRamp(){
  /* As the robot is front wheel driven it loses the line
  going over the ramp. So, we switch to navigation using
  an ultrasonic and distance to the wall */
  blink();
  distance = readUSRight();
  if (distance > rampDist){ // unlike the turn functions above we do not bother checking if the speed or direction need updating
    myMotorLeft->setSpeed(baseMotorSpeed); // as the distance is continuous, rather than discrete, so it is very likely an update is needed
    myMotorRight->setSpeed(USnavSpeed); // rampDist was measured based on the robot dimensions and US placement
    oldLeftSpeed = baseMotorSpeed; // too far from the wall we turn right (always counterclockwise around the table)
    oldRightSpeed = USnavSpeed;
    myMotorLeft->run(FORWARD);
    myMotorRight->run(FORWARD);
  }
  if (distance > 20){ // going over the ramp sometimes the wall is not seen, so we want to just go full speed ahead
    myMotorLeft->setSpeed(baseMotorSpeed); // as this is the steepest part of the ramp
    myMotorRight->setSpeed(baseMotorSpeed);
    oldLeftSpeed = baseMotorSpeed;
    oldRightSpeed = baseMotorSpeed;
    myMotorLeft->run(FORWARD);
    myMotorRight->run(FORWARD);
  }
  if (distance < rampDist) { // to close to the wall we want to turn left (always counterclockwise aroudn the table)
    myMotorRight->setSpeed(baseMotorSpeed);
    myMotorLeft->setSpeed(USnavSpeed);
    oldLeftSpeed = USnavSpeed;
    oldRightSpeed = baseMotorSpeed;
    myMotorLeft->run(FORWARD);
    myMotorRight->run(FORWARD);
  }
  if (distance == rampDist) { // if the right distance from the wall we want to go straight
    myMotorRight->setSpeed(baseMotorSpeed);
    myMotorLeft->setSpeed(baseMotorSpeed);
    oldLeftSpeed = baseMotorSpeed;
    oldRightSpeed = baseMotorSpeed;
    myMotorLeft->run(FORWARD);
    myMotorRight->run(FORWARD);
  } 
  return;
}

void USNavTunnel(){
  /* with no line in the tunnel, it also makes sense
  to use an ultrasonic navigation method with distance to
  the wall here too */
  distance = readUSRight();
  if (distance > tunnelDist){ // tunnelDist was measured based on robot dimensions and US placement
    myMotorLeft->setSpeed(baseMotorSpeed); // too far from wall we turn left
    myMotorRight->setSpeed(USnavSpeed);
    oldLeftSpeed = baseMotorSpeed;
    oldRightSpeed = USnavSpeed;
    myMotorLeft->run(FORWARD);
    myMotorRight->run(FORWARD);
  }
  if (distance < tunnelDist) { // too close to wall we turn right
    myMotorRight->setSpeed(baseMotorSpeed);
    myMotorLeft->setSpeed(USnavSpeed);
    oldLeftSpeed = USnavSpeed;
    oldRightSpeed = baseMotorSpeed;
    myMotorLeft->run(FORWARD);
    myMotorRight->run(FORWARD);
  }
  if (distance == tunnelDist) { // correct distance we go forwards
    myMotorRight->setSpeed(baseMotorSpeed);
    myMotorLeft->setSpeed(baseMotorSpeed);
    oldLeftSpeed = baseMotorSpeed;
    oldRightSpeed = baseMotorSpeed;
    myMotorLeft->run(FORWARD);
    myMotorRight->run(FORWARD);
  } 
  return;
}

void rampNav(){
  /* using our USNavRamp() function we now check when to
  switch to this function by checking when we are close
  to the wall */
  distance = readUSRight();
  if (distance < rampDist+2){ // (US sensor at angles behave strangely, so rampDist+2 is our check))
    leftTurn(); // we still need to finish the turn at this point
    delay(600);
    while (true){ // continue to execture rampNav until off the ramp
      USNavRamp();
      leftOuter = digitalRead(leftLineSensorOuter);
      if (leftOuter == 1 && distance > rampDist - 4 && distance < rampDist + 4){ // we are off the ramp when we reach the left turn indicated by the line
        while (true){ // the distance check is to ensure nothing strange happens on the ramp (the robot occasionally veered going over the ramp)
          sharpLeft(); // execute a left turn until
          rightInner = digitalRead(rightLineSensorInner);
          if (rightInner == 1){ // we know we are back on the line
            break;
          }
        }
      break;
      }
    }
  }
  return;
}

void tunnelNav(){
  /* using our USNavRamp() function we now check when to
  switch to this function by checking when we are close
  to the wall. Exactly the same as the rampNav, but
  with different values), and distance checks for returning
  to line following removed as unnecessary */
  distance = readUSRight();
  if (distance < tunnelDist + 2){
    leftTurn();
    delay(600);
    while (true){
      USNavTunnel();
      leftOuter = digitalRead(leftLineSensorOuter);
      if (leftOuter == 1){
        while (true){  
          sharpLeft();
          rightInner = digitalRead(rightLineSensorInner);
          if (rightInner == 1){
            break;
          }
        }
      break;
      }
    }
  }
  return;
}

// junction Checking functions

void rightJunctionCheck(){
  /* detects right junctions, incrementing the junctionCount.
  If the junctionCount is then the junction Needed based on 
  current block density, the junction is entered, and the block
  delivered. If the junction entered is the home junction, the program
  ends as the robot has returned */
  rightOuter = digitalRead(rightLineSensorOuter);
  if (rightOuter == 1){
    junctionCount =  (junctionCount + 1) % 6; // increment junctionCount so the robot knows where it is
    if (junctionCount == junctionNeeded && junctionCount != 3){ // if a right turning is detected and we need to turn in (robot goes counter-clockwise so all boxes on the right)
      // however, we never want to turn at junction 3, as the block is placed on the centre
      backwards(); // small adjustment that improves accuracy of turn
      delay(100);
      sharpRight(); // begin the turn
      delay(ninetyTurnDelay); // we are turning until our inner left sensor sees the line again (the orthogonal to followed line). We want to ignore the line we are already on hence the delay
      while (true){
        leftInner = digitalRead(leftLineSensorInner);
        if (leftInner == 1){ // continue turning until the left sensor sees the turning line (every junction includes a white line)
          stop();
          break;
        }
      }
      while (true){
        lineFollowing(); // we use lineFollowing to ensure we enter the box straight
        leftOuter = digitalRead(leftLineSensorOuter);
        if (leftOuter == 1){ // end line following when the line no longer exists
          break;
        }
      }
      forwards(); // enters the box
      delay(1000);
      stop();
      if (junctionNeeded == 0){ // if we are back at the starting box
        forwards();
        delay(700);      
        stop();
        exit(0); // we have returned to the start box and can stop running code
      }
      dropBlock(); // opens grabber, delivering block
      hasBlock = 0;
      blocksCollected += 1;
      backwards(); // reverse until the main white line is refound
      int tempcount = 0; // dont want to turn on first orthoganl seen    
      while (true){
        leftOuter = digitalRead(leftLineSensorOuter);
        if (leftOuter == 1){
          tempcount = tempcount + 1;
          delay(350); // only detect the orthogonal line once while passing it
        }
        if (tempcount == 2){ // two orthogonal lines
          break;
        }
      }
      forwards(); // adjustment to improve turn, and counteract above motion
      delay(300);
      sharpLeft(); // turn back to continue on the course
      delay(ninetyTurnDelay);
      while (true){
        rightInner = digitalRead(rightLineSensorInner);
        if (rightInner == 1){ // turn until back on main line
          break;
        }
      }
    }
    forwards(); // continue on before a junction can be checked for again, to avoid the same junction being detected twice
    delay(200);
    }
  return;
}

void leftJunctionCheck(){
  /* detects left junctions, incrementing the junctionCount.
  If the junctionCount is then the junction Needed based on 
  where the next block is, the junction is entered, and the block
  collected. With the next junction needed assigned */
  leftOuter = digitalRead(leftLineSensorOuter);
  if (leftOuter == 1){
    junctionCount = (junctionCount + 1) % 6;
    if (junctionCount == junctionNeeded && junctionCount != 3){ // if a left turning is detected and we need to turn there
    // however, we never want to turn at junction 3, as the block is placed on the centre
      backwards(); // adjustment improves accuracy
      delay(100);
      sharpLeft(); // begin the turn
      delay(ninetyTurnDelay); // we are turning until our inner right sensor sees the line again (the orthogonal to followed line). We want to ignore the line we are already on hence the delay
      while (true){
        rightInner = digitalRead(rightLineSensorInner);
        if (rightInner == 1){ // continue turning until the left sensor sees the turning line (every junction includes a white line)
          stop();
          break;
        }
      }
      while (true){
        lineFollowing(); // follow the line until we detect our block
        sensorValue = readIR();
        Serial.println(sensorValue);
        if (sensorValue > 320){ // block has been detected, so now need to identify it
          break;
        }
      }
      forwards(); // block detection works best slightly closer to the block, but the IR readings are poor too close
      delay(600);
      stop();
      blockIdentification(); // identifies the block, and picks it up
      backwards(); // reverse until the main white line is refound   
      while (true){
        leftOuter = digitalRead(leftLineSensorOuter);
        if (leftOuter == 1){
          break; // want to reverse until we refind the following line, only one orthogonal line here
        }
      }
      forwards(); // adjustment improves turn quality
      delay(200);
      sharpRight(); // turn back to continue on the course
      delay(ninetyTurnDelay);
      while (true){
        leftInner = digitalRead(leftLineSensorInner);
        if (leftInner == 1){
          break;
        }
    }  
    forwards(); // continue on before a junction can be checked for again, to avoid the same junction being detected twice
    delay(200);
    }
  }
  return;
}

// block functions
void blockMeasureLeft(){
  /* does a series of short motions to the left, to take multiple
  readings for the block, improving accuracy of identification */
  for (int i = 1; i < 5; i++){ // 5 readings turning left
    leftTurn();
    delay(50);
    stop();
 
    delay(100);
    distance = readUSFront(); // take US reading
    if(distance > 40 && distance < 60){ // if reading behaves strangely, block is high density
      blockDensity = 1;
    }
  }
  return;  
}

void blockMeasureRight(){
  /* does a series of short motions to the right, to take multiple
  readings for the block, improving accuracy of identification */
  for (int i = 1; i < 10; i++){ // 10 readings turning right. See blockIdentification() and blockMeasureLeft()
    rightTurn();
    delay(50);
    stop();
 
    delay(100);
    distance = readUSFront(); // read US sensor
    Serial.println(distance);
    if( distance > 40 && distance < 60){ // if reading behaves strangely, block is high density
      blockDensity = 1;
    }
  }
  return;
}

void blockLEDs(){
  /* based on the block density lights up the relevant
  LED, see blockIdentification */
  if(blockDensity == 0){
    digitalWrite(lowDensityLED, HIGH);
    digitalWrite(highDensityLED, LOW);
    Serial.println("low density");  
  }else{
    digitalWrite(highDensityLED, HIGH);
    digitalWrite(lowDensityLED, LOW);
    Serial.println("high density");
  }
  return;
}

void blockIdentification(){
  /* Using the IR sensor we detect if there is a block, then
  using the US sensor, we identify what type of block it is */
  sensorValue = readIR();

  if( sensorValue > 320 && hasBlock != 1){ // if there is a block and we dont have one already
    digitalWrite(lowDensityLED, LOW);
    digitalWrite(highDensityLED, LOW);

    blockMeasureLeft(); // we want to turn the robot slightly to improve the accuracy of our detection
    blockMeasureRight(); // so we turn left a small amount, then right a small amount, then left again
    blockMeasureLeft(); // to return to a roughly straight line

    blockLEDs();

    if (blockDensity == 0){
      junctionNeeded = 5; // need to deliver lowe density blocks to the green box  
    }
    else{
      junctionNeeded = 1; // need to deliver high density blocks to the red box
    }

    blockDensity = 0; // assume block density low, unless we detect it is high
    delay(5000); // specified pause

    collectBlock(); // closes grabber, collecting block
    hasBlock = 1;
  }
  return;
}

void dropBlock(){
  /* sets the servo position to 180 i.e. openPos */
  if (currentPos != openPos){
    for (int i = closedPos; i <= openPos; i++){
    myservo.write(i);
    delay(10);
    }
    currentPos = openPos;
  }
  digitalWrite(lowDensityLED, LOW);
  digitalWrite(highDensityLED, LOW);  
  return;
}

void collectBlock(){
  /* sets the servo position to 0 i.e. closedPos */
  if (currentPos != closedPos){
    for (int i = openPos; i >= closedPos; i--){
    myservo.write(i);
    delay(10);
    }
  currentPos = closedPos;
  }
  return;
}

void nextBlockJunction(){
  /* based on the number of blocks collected
  tells the robot where to go for the next block */
  if (hasBlock == 0){ // we run this function every loop, but only want to update with no block
    if (blocksCollected == 1){ // middle block collected, collect block at first left junction
      junctionNeeded = 2;
    }
    else if (blocksCollected == 2){ // middle and first left collected, collect block at second left junction
      junctionNeeded = 4;
    }
    else if (blocksCollected > 2){ // first 3 collected, no time for 4th, return to start
      junctionNeeded = 0;
    }
  }
}

void setup() {
  AFMS.begin();
  IMU.begin();
  Serial.begin(9600);
  myMotorLeft->setSpeed(127); //left wheel
  myMotorRight->setSpeed(127);// right wheel
  oldLeftSpeed = 127;
  oldRightSpeed = 127;

  // button setup
  pinMode(onOff, INPUT); // push to make button

  // line sensors setup
  pinMode(leftLineSensorOuter, INPUT);
  pinMode(leftLineSensorInner, INPUT); 
  pinMode(rightLineSensorInner, INPUT); 
  pinMode(rightLineSensorOuter, INPUT);

  // ultrasonic sensors setup
  pinMode(echoPinRight, INPUT);
  pinMode(trigPinRight, OUTPUT);
  pinMode(echoPinFront, INPUT); 
  pinMode(trigPinFront, OUTPUT);

  // LED setup
  pinMode(blinkLED, OUTPUT);
  pinMode(highDensityLED, OUTPUT);
  pinMode(lowDensityLED, OUTPUT);

  // servo setuo
  myservo.attach(servoPin);
  
  for (int i = closedPos; i <= openPos; i++){ // ensure servo is in the open position
    myservo.write(i);
    delay(10);
  }
  currentPos = openPos;


  begin();
  forwards(); // exit the box, these delays tested for
  delay(1000); // this exit process was hardcoded to save time
  rightTurn(); // going to the following line and turning took
  delay(2500);  // longer than this method
  stop();
}

void loop() {
  // put your main code here, to run repeatedly:
  begin(); // begin check before continuing
  lineFollowing(); // make sure to follow the line, unless ramp or tunnel navigation needed
  leftJunctionCheck(); // always checking for left and right junctions
  rightJunctionCheck();
  if (junctionCount == 1){ // only want to check for rampNav between the red box and first left junction
    rampNav();
  }
  else if (junctionCount == 4){ // only want to check for tunnelNav between the second left junction and green box
    tunnelNav();
  }
  if (junctionCount == 3 && junctionNeeded == 3){ // cannot use junctionCount == junctionNeeded as this will do for non-3 values. 3 is a special case as block on the middle
    blockIdentification(); // unlike the other blocks, this check cannot be put inside the relevant junction function
  }
  nextBlockJunction(); // update the junction we need to go to, if necessary
  finish(); // for stopping the program if any issues occur
}