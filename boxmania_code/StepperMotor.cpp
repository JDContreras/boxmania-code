#include "StepperMotor.h"

StepperMotor::StepperMotor(const StepperConfig& config)
  : stepPin(config.stepPin), dirPin(config.dirPin), speed(40), stallPin(config.stallPin), stepToDistanceFactor(80), currentState(MotorState::DISABLE) {
  // Constructor implementation
  // Initialize digital pins
  pinMode(config.stepPin, OUTPUT);
  pinMode(config.dirPin, OUTPUT);
  pinMode(config.stallPin, INPUT);
  driver.setup(config.serialPort, 500000);
  driver.setRunCurrent(config.current);
  driver.setStallGuardThreshold(config.stallThreshold);
  driver.enableAutomaticCurrentScaling();
  lastStepTime = millis(); // Initialize lastStepTime
  initialSpeed = 10.0;
  acceleration = 50.0;
}

void StepperMotor::move(float distance) {
  // move implementation
  float currentSpeed = initialSpeed; // Start at the initial speed
  float currentDistance = 0; // Initialize the distance traveled

  // Calculate time needed for acceleration to reach the target speed
  float accelerationTime = (speed - initialSpeed) / acceleration;

  // Calculate distance covered during acceleration phase
  float accelerationDistance = 0.5 * (initialSpeed + speed) * accelerationTime;

  // Calculate the distance for constant speed phase
  float constantSpeedDistance = distance - 2 * accelerationDistance;

  // Calculate the time for constant speed phase
  float constantSpeedTime = constantSpeedDistance / speed;

  // Calculate the total move time
  float totalTime = 2 * accelerationTime + constantSpeedTime;

  // Calculate the number of steps 
  unsigned long totalSteps = mmToPulses(distance);

  // Calculate the number of steps for acceleration phase
  unsigned long accelerationSteps = mmToPulses(accelerationDistance);

  // Calculate the number of steps for constant speed phase
  unsigned long constantSpeedSteps = mmToPulses(constantSpeedDistance);

  // Calculate the number of steps for deceleration phase
  unsigned long decelerationSteps = totalSteps - accelerationSteps - constantSpeedSteps;

  // Step generation loop
  for (unsigned long step = 0; step < totalSteps; step++) {
    if (step < accelerationSteps) {
      // Acceleration phase: Adjust speed and step interval
      currentSpeed += acceleration * (totalTime / totalSteps);
    } else if (step >= accelerationSteps && step < accelerationSteps + constantSpeedSteps) {
      // Constant speed phase: Maintain target speed
      currentSpeed = speed;
    } else {
      // Deceleration phase: Adjust speed and step interval
      currentSpeed -= acceleration * (totalTime / totalSteps);
    }

    int stepInterval = int(1000000.0 / currentSpeed * stepToDistanceFactor);
    PORT->Group[g_APinDescription[stepPin].ulPort].OUTSET.reg = (1ul << g_APinDescription[stepPin].ulPin);
    // Wait for 10 microseconds (you can adjust the delay to match your requirements)
    delayMicroseconds(10);
    // Set stepPin low
    PORT->Group[g_APinDescription[stepPin].ulPort].OUTCLR.reg = (1ul << g_APinDescription[stepPin].ulPin);
    delayMicroseconds(stepInterval-10);
    //currentDistance += stepToMm(1); // Update current distance based on the step
  }  
}

uint32_t StepperMotor::mmToPulses(float distance) {
  return (uint32_t)(distance * stepToDistanceFactor);
}

void StepperMotor::enable() {
  // enable implementation
  bool hardware_disabled = driver.hardwareDisabled();
  if (!hardware_disabled){
    driver.enable();
    currentState = MotorState::STANDSTILL;
  }

}

void StepperMotor::pulse() {
  // Set stepPin high
  PORT->Group[g_APinDescription[stepPin].ulPort].OUTSET.reg = (1ul << g_APinDescription[stepPin].ulPin);
  // Wait for 10 microseconds (you can adjust the delay to match your requirements)
  delayMicroseconds(10);
  // Set stepPin low
  PORT->Group[g_APinDescription[stepPin].ulPort].OUTCLR.reg = (1ul << g_APinDescription[stepPin].ulPin);
}

void StepperMotor::home() {
  // home implementation
  if (currentState == MotorState::STANDSTILL){
    currentState = MotorState::HOMING;
  }
}

void StepperMotor::disable() {
  // disable implementation
  driver.disable();
  currentState = MotorState::DISABLE;
}

void StepperMotor::setSpeed(int newSpeed) {
  // setSpeed implementation
  speed = newSpeed;
}

void StepperMotor::setCurrent(int current) {
  // setCurrent implementation
  driver.setRunCurrent(current);
}

void StepperMotor::setStallThreshold(int threshold) {
  // setStallThreshold implementation
  driver.setStallGuardThreshold(threshold);
}

MotorState StepperMotor::getState() {
  return currentState;
}