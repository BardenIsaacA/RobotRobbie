const int triggerPin = 12; // Change this pin number to whichever the trig is connected to, named pin for ease of reading
const int echoPin = 13; // Change this pin number top whichever the echo is connected to, named pin for ease of reading
// both are set as constant as they are pin numbers

// defines variables
long duration; // duration stored in long as is measured in microseconds
int distance; // distance stored as int as measured in cm up to 994 (maximum for sensor)
void setup() {
  pinMode(triggerPin, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin, INPUT); // Sets the echoPin as an Input
  Serial.begin(9600); // Starts the serial communication
}
void loop() {
  // Clears the trigPin
  digitalWrite(triggerPin, LOW);
  delay(1000);
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(triggerPin, HIGH);
  delayMicroseconds(100);
  digitalWrite(triggerPin, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echoPin, HIGH);
  // Calculating the distance
  distance = duration * 0.034 / 2; // the time recorded is the time in microseconds for the sound to go to and from the wall hence divided by 2 
  // Speed of sound in air is 340 m/s or 0.034cm/us or 0.34mm/us
  // Prints the distance on the Serial Monitor
  Serial.print("Distance: ");
  Serial.println(distance);
}