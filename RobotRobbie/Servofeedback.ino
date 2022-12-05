
const int  InfradSensor=5 ;
const int  UltrasonicSensor=6;
int Blockcollected=0;
int collectedfactorInfrad=0;
int collectedfactorUltra=0;

void setup() {
  pinMode(InfradSensor, INPUT);
  pinMode(UltrasonicSensor, INPUT);

}

void Servofeedback(){
  collectedfactorInfrad = digitalRead(InfradSensor);
  if (collectedfactorInfrad==1 ){
    Blockcollected=1;
  }
}
