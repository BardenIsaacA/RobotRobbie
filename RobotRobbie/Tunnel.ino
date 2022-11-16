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

const int triggerPin = 12; // Change this pin number to whichever the trig is connected to
const int echoPin = 13; // Change this pin number top whichever the echo is connected to

long duration = 0; // duration stored in long as is measured in microseconds
int distance = 0; // distance stored as int as measured in cm up to 994 (maximum for sensor)

void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:

}
