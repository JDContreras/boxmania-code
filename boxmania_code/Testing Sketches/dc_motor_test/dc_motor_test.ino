//Pins definition
#define F_PIN 9
#define R_PIN  10
void setup() {
  //pins initialization
  pinMode(F_PIN, OUTPUT);
  pinMode(R_PIN, OUTPUT);
  Serial.begin(1000000);
}

void loop()
{
  if(Serial.available() > 0)  { //wait for message
    int vel = Serial.parseInt(); //parse as int
    Serial.println(vel);
    if (vel > 0 and vel < 255) { //rotate forward
      uint32_t velF = vel;
      analogWrite(R_PIN,0);
      analogWrite(F_PIN,0);
      delay(200); //wait to stop
      analogWrite(F_PIN,velF);
    }
    else if (vel < 0 and vel > -255){ //reverse
      uint32_t velF = abs(vel);
      analogWrite(R_PIN,0);
      analogWrite(F_PIN,0);
      delay(200); //wait to stop
      analogWrite(R_PIN,velF);
    }
    else{ //stop
      analogWrite(R_PIN,0);
      analogWrite(F_PIN,0);
    }
 }
}

