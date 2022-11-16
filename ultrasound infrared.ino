
#define echoPin 3 // attach pin D2 Arduino to pin Echo of HC-SR04
#define trigPin 2 //attach pin D3 Arduino to pin Trig of HC-SR04
long duration; // variable for the duration of sound wave travel
int distance; // variable for the distance measurement
int sensorPin = A0;    // select the input pin for the potentiometer  
int sensorValue = 0;  // variable to store the value coming from the sensor
void setup() {
  // declare the ledPin as an OUTPUT:

  Serial.begin(9600);
   pinMode(trigPin, OUTPUT); // Sets the trigPin as an OUTPUT
  pinMode(echoPin, INPUT); // Sets the echoPin as an INPUT
  pinMode(1, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(13, OUTPUT);
  pinMode(12, OUTPUT);
  pinMode(11, OUTPUT);
  digitalWrite(1, HIGH);
  digitalWrite(4, LOW);
  digitalWrite(11, LOW);
}
void loop() {
 
  sensorValue = analogRead(sensorPin);
  Serial.println(sensorValue);


  // stop the program for for <sensorValue> milliseconds:
  delay(sensorValue);
   // Clears the trigPin condition
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  // Sets the trigPin HIGH (ACTIVE) for 10 microseconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echoPin, HIGH);
  // Calculating the distance
  distance = duration * 0.034 / 2; // Speed of sound wave divided by 2 (go and back)
  // Displays the distance on the Serial Monitor
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");
  if( sensorValue > 350){ //350 also works
    if(distance < 20){
    digitalWrite(13, HIGH);
  }else{
  digitalWrite(13, LOW);}
  if(distance > 20){
  digitalWrite(12, HIGH);
  }else{
  digitalWrite(12, LOW);}
  }
  else{
  digitalWrite(13, LOW);
  digitalWrite(12, LOW);}
}