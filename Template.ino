// This is a template containing all the imports and constants currently needed to run the code
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

void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:

}
