#include "StepperMotor.h"

StepperMotor::StepperMotor(const StepperConfig& config)
  : stepPin(config.stepPin), 
    dirPin(config.dirPin), 
    speed(40),
    initialSpeed(config.limits.minVelocity), 
    currentState(MotorState::DISABLE), 
    limits(config.limits), 
    homingDirection(config.homing.direction), 
    homingVel(config.homing.velocity), 
    lastStepTime(millis()),  // Initialize lastStepTime
    acceleration(50.0),  // Default acceleration value
    stepPerMilimeter(config.stepPerMilimeter),
    stallPort(portInputRegister(digitalPinToPort(config.stallPin))),
    stallBit(digitalPinToBitMask(config.stallPin)),
    axishomed(false)
  {
  // Initialize digital pins
  pinMode(config.stepPin, OUTPUT);
  pinMode(config.dirPin, OUTPUT);
  pinMode(config.stallPin, INPUT);

  // Setup the TMC2209 driver
  driver.setup(config.serialPort, 500000); //max baudrate

  // Additional setup for the TMC2209 driver
  driver.setRunCurrent(config.current);
  driver.setStallGuardThreshold(config.stallThreshold);
  driver.enableAutomaticCurrentScaling();
}

moveResult StepperMotor::moveRelative(float distance) {
  // move implementation
  moveResult result;
  result.complete = true; // Assume the movement is complete unless stalled

  if (speed < initialSpeed) {
    speed = initialSpeed;
  }

  // Calculate the maximum achievable speed based on acceleration and distance
  float maxAchievableSpeed = sqrt(2 * acceleration * distance);

  // Ensure that the requested speed is achievable
  if (speed > maxAchievableSpeed) {
    speed = maxAchievableSpeed;
  }

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
  
  unsigned long reachedSteps = 0;

  // Step generation loop
  for (unsigned long step = 0; step < totalSteps; step++) {
    if (stallStatus()) {
        // Stall detected
        result.complete = false;
        break; // Exit the loop if a stall occurs
    }
    

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
    int stepInterval = int(1000000.0 / currentSpeed * stepPerMilimeter);
    pulse(stepInterval);

    reachedSteps++;

  }

  if (result.complete) {
    // If no stall was detected, set the result distance to the requested distance
    result.distance = distance;
  } 
  else {
    // If a stall was detected, set the result distance based on reached steps
    result.distance = static_cast<float>(reachedSteps) / stepPerMilimeter;
  }
  currentPosition = currentPosition + result.distance;
  return result;
}

bool StepperMotor::stallStatus() {
  return (bool)((*stallPort & stallBit) != 0);
}

void StepperMotor::moveAbs(float targetPosition) {
  // Calculate the relative distance to move from the current position to the target position
  targetPosition = constrain(targetPosition, limits.minPosition, limits.maxPosition);

  float relativeDistance = targetPosition - currentPosition;
  // Call moveRelative to execute the motion
  moveRelative(relativeDistance);

  // Update the current position if the motion is successful
  currentPosition = targetPosition;
}

uint32_t StepperMotor::mmToPulses(float distance) {
  return (uint32_t)(distance * stepPerMilimeter);
}

void StepperMotor::enable() {
  // enable implementation
  bool hardware_disabled = driver.hardwareDisabled();
  if (!hardware_disabled){
    driver.enable();
    currentState = MotorState::STANDSTILL;
  }
}

void StepperMotor::pulse(int stepInterval) {
  // Set stepPin high
  PORT->Group[g_APinDescription[stepPin].ulPort].OUTSET.reg = (1ul << g_APinDescription[stepPin].ulPin);
  // Wait for 10 microseconds (you can adjust the delay to match your requirements)
  delayMicroseconds(10);
  // Set stepPin low
  PORT->Group[g_APinDescription[stepPin].ulPort].OUTCLR.reg = (1ul << g_APinDescription[stepPin].ulPin);
  delayMicroseconds(stepInterval-10);
}

void StepperMotor::home() {
  // home implementation
  int stepInterval = int(1000000.0 / homingVel * stepPerMilimeter);
  if (currentState == MotorState::STANDSTILL){
    currentState = MotorState::HOMING;
    if (homingDirection){
      digitalWrite(dirPin,HIGH);
      while (!stallStatus()) { //TODO: this is locking, do validation
        pulse(stepInterval);

      }
      currentPosition = limits.maxPosition;
    }
    else{
      digitalWrite(dirPin,HIGH);
      while (!stallStatus()) { //TODO: this is locking, do validation
        pulse(stepInterval);

      }
      currentPosition = limits.minPosition;
    }
  }
  axishomed = true;
}

void StepperMotor::disable() {
  // disable implementation
  driver.disable();
  currentState = MotorState::DISABLE;
  axishomed = false;
}

void StepperMotor::setSpeed(int newSpeed) {
  // Check if the new speed exceeds the maximum velocity limit
  if (newSpeed > limits.maxVelocity) {
    speed = limits.maxVelocity;
  } else if (newSpeed < limits.minVelocity) { // Check if the new speed is below the minimum velocity limit
    speed = limits.minVelocity;
  } else {
    // The new speed is within the allowed limits
    speed = newSpeed;
  }
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