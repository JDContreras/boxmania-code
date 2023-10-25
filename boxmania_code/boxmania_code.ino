#include "StateMachine.h"
#include "StepperMotor.h"
StateMachine stateMachine;

void setup() {
  // Arduino setup code here
}

void loop() {
  stateMachine.update();
  // Other loop code here
}