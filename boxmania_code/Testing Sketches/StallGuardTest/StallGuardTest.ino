#include <TMC2209.h>
const long SERIAL_BAUD_RATE = 250000;
const int DELAY = 2000;
// current values may need to be reduced to prevent overheating depending on
// specific motor and power supply voltage
const uint8_t RUN_CURRENT_PERCENT = 50;
const int32_t VELOCITY = 50000;
const uint8_t STALL_GUARD_THRESHOLD = 100;
int stepPin = MISO;
int dirPin = MOSI;
int enPin = SCK;
int stallPin = A4;
// Instantiate TMC2209
TMC2209 stepper_driver;
int count;
void setup(){
  Serial.begin(1000000);
  stepper_driver.setup(Serial1,SERIAL_BAUD_RATE);
  delay(DELAY);
  stepper_driver.setStallGuardThreshold(STALL_GUARD_THRESHOLD);
  stepper_driver.disableCoolStep();
  stepper_driver.disableStealthChop();
  stepper_driver.setCoolStepDurationThreshold(1000000);
  stepper_driver.setStealthChopDurationThreshold(10);
  delay(DELAY);
  //stepper_driver.enableCoolStep();
  stepper_driver.enableStealthChop();
  stepper_driver.enableAutomaticCurrentScaling();
  stepper_driver.setRunCurrent(RUN_CURRENT_PERCENT);
  stepper_driver.setStallGuardThreshold(STALL_GUARD_THRESHOLD);
  stepper_driver.setMicrostepsPerStep(8);
  stepper_driver.enable();
  //stepper_driver.moveUsingStepDirInterface();
  //stepper_driver.moveAtVelocity(VELOCITY);
  pinMode(stallPin, INPUT);
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(enPin, OUTPUT);
  digitalWrite(dirPin,LOW);
  digitalWrite(enPin,HIGH);
  delay(100);
  digitalWrite(enPin,LOW);
}

void loop(){
  /*
  if (not stepper_driver.isSetupAndCommunicating()){
    Serial.println("Stepper driver not setup and communicating!");
    return;
  }
  */
  
  //10 step pin
  count++;
  digitalWrite(stepPin,HIGH);
  delayMicroseconds(10);
  digitalWrite(stepPin,LOW);
  delayMicroseconds(500);

  if (digitalRead(stallPin)){
    Serial.println("stall");
    digitalWrite(enPin,HIGH);
    digitalWrite(dirPin, digitalRead(dirPin) == LOW ? HIGH : LOW);
    delay(1000);
    digitalWrite(SCK,LOW);
  }
  /*
  uint16_t stall_guard_result = stepper_driver.getStallGuardResult();
  Serial.println(stall_guard_result);
   
  if (count>100){
    
    Serial.print("step time   ");
    Serial.println(stepper_driver.getInterstepDuration());
    Serial.print("stall guard   ");
    Serial.println(stepper_driver.getStallGuardResult());
    count = 0;
  }
  */
 

}
