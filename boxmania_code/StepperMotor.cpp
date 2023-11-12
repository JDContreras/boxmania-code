#include "StepperMotor.h"

StepperMotor::StepperMotor(const StepperConfig& config)
  : stepPin(config.stepPin), 
    dirPin(config.dirPin), 
    enablePin(config.enPin),
    speed(40),
    initialSpeed(config.limits.minVelocity), 
    currentState(MotorState::DISABLE), 
    stallThreshold(config.stallThreshold),
    motorCurrent(config.current),
    limits(config.limits), 
    homingDirection(config.homing.direction), 
    homingStepInterval(int(1000000.0 / (config.homing.velocity * config.stepPerMilimeter))), 
    lastStepTime(millis()),  // Initialize lastStepTime
    acceleration(50.0),  // Default acceleration value
    stepPerMilimeter(config.stepPerMilimeter),
    stallPort(portInputRegister(digitalPinToPort(config.stallPin))),
    stallBit(digitalPinToBitMask(config.stallPin)),
    axishomed(false),
    homingState(0),
    lastMicros(0),
    maxPulseCount((limits.maxPosition-limits.minPosition)*config.stepPerMilimeter),
    pulseCount(0),
    totalPulseCount(0),
    prevExecute(false),
    //driver(config.driver),
    serialPort(config.serialPort)
  {
  
}

bool StepperMotor::configDriver(){
  pinMode(stepPin, OUTPUT);
  pinMode(enablePin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  driver.setup(serialPort, 250000); //max baudrate
  delay(100);
  if (driver.isSetupAndCommunicating()){
    driver.setHardwareEnablePin(-1);
    driver.setStallGuardThreshold(stallThreshold);
    driver.setRunCurrent(motorCurrent);
    driver.setStandstillMode(driver.NORMAL);
    driver.setHoldCurrent(50);
    driver.setMicrostepsPerStep(8); //TODO ad argument for this

    driver.disableCoolStep();
    driver.disableStealthChop();
    driver.setCoolStepDurationThreshold(1000000);
    driver.setStealthChopDurationThreshold(10);
    delay(100);
    driver.enableStealthChop();
    driver.enableAutomaticCurrentScaling();
    driver.enable();
    Serial.println("driver configured");
    return true;
  }
  else {
    Serial.println("driver no conected");
    return false;
  }

}

float StepperMotor::getCurrentPosition(){
  return currentPosition;
}

MoveResult StepperMotor::moveRelative(float distance) {
  
  // move implementation
  MoveResult result;
  int tempSpeed;
  float tempDistance;
  result.complete = true; // Assume the movement is complete unless stalled


  if (speed < initialSpeed) {
    tempSpeed = initialSpeed;
  }
  else{
    tempSpeed = speed;
  }
  //set direction
  digitalWrite(dirPin, distance >= 0 ? HIGH : LOW);
  //use distance as positive for next operations
  tempDistance = abs(distance);

  // Calculate the maximum achievable speed based on acceleration and distance
  float maxAchievableSpeed = sqrt((2 * acceleration * (tempDistance/2)) + (initialSpeed*initialSpeed));

  // Ensure that the requested speed is achievable
  if (tempSpeed > maxAchievableSpeed) {
    tempSpeed = maxAchievableSpeed;
  }

  float currentSpeed = initialSpeed; // Start at the initial speed
  float currentDistance = 0; // Initialize the distance traveled

  // Calculate time needed for acceleration to reach the target speed
  float accelerationTime = (tempSpeed - initialSpeed) / acceleration;
  // Calculate distance covered during acceleration phase
  float accelerationDistance = 0.5 * (initialSpeed + tempSpeed) * accelerationTime;
  // Calculate the distance for constant speed phase
  float constantSpeedDistance = tempDistance - 2 * accelerationDistance;

  // Calculate the time for constant speed phase
  float constantSpeedTime = constantSpeedDistance / tempSpeed;

  // Calculate the total move time
  float totalTime = 2 * accelerationTime + constantSpeedTime;

  // Calculate the number of steps 
  unsigned long totalSteps = mmToPulses(tempDistance);

  // Calculate the number of steps for acceleration phase
  unsigned long accelerationSteps = mmToPulses(accelerationDistance);

  // Calculate the number of steps for constant speed phase
  unsigned long constantSpeedSteps = mmToPulses(constantSpeedDistance);

  // Calculate the number of steps for deceleration phase
  unsigned long decelerationSteps = totalSteps - accelerationSteps - constantSpeedSteps;
  
  unsigned long reachedSteps = 0;
  // Step generation loop
  int count = 0;
  float timeFraction = (totalTime/totalSteps);
  for (unsigned long step = 0; step < totalSteps; step++) {
    if (stallStatus()) {
        // Stall detected
        result.complete = false;
        toggleEnablePin();
        break; // Exit the loop if a stall occurs
    }
    
    if (step < accelerationSteps) {
  // Acceleration phase: Adjust speed and step interval
      //currentSpeed = initialSpeed + acceleration * totalTime*( step / totalSteps);
      currentSpeed = initialSpeed + acceleration * (timeFraction)* step;
    } else if (step >= accelerationSteps && step < accelerationSteps + constantSpeedSteps) {
      // Constant speed phase: Maintain target speed
      currentSpeed = tempSpeed;
    } else {
      // Deceleration phase: Adjust speed and step interval
      currentSpeed = tempSpeed - acceleration * (timeFraction) * (step - accelerationSteps - constantSpeedSteps);
    }
    int stepInterval = int(1000000.0 / (currentSpeed * stepPerMilimeter));
    pulse(stepInterval);
    count++;
    reachedSteps++;

  }

  if (result.complete) {
    // If no stall was detected, set the result distance to the requested distance
    result.distance = distance;
  } 
  else {
    // If a stall was detected, set the result distance based on reached steps
    result.distance = static_cast<float>(reachedSteps) / stepPerMilimeter;
    result.distance = (distance >= 0) ? result.distance : -result.distance;
  }
  currentPosition = currentPosition + result.distance;
  
  return result;
  
}

void StepperMotor::moveAbs(float targetPosition) {
  // Calculate the relative distance to move from the current position to the target position
  targetPosition = constrain(targetPosition, limits.minPosition, limits.maxPosition);

  float relativeDistance = targetPosition - currentPosition;
  Serial.print("moving: ");
  Serial.print(relativeDistance);
  Serial.println("mm");
  // Call moveRelative to execute the motion
  moveRelative(relativeDistance);

  // Update the current position if the motion is successful
  //currentPosition = targetPosition;
}

uint32_t StepperMotor::mmToPulses(float distance) {
  return (uint32_t)(distance * stepPerMilimeter);
}

void StepperMotor::enable() {
  // enable by hardware and software
  driver.enable();
  digitalWrite(enablePin, LOW);
  currentState = MotorState::STANDSTILL;
}

void StepperMotor::disable() {
  // disable implementation
  digitalWrite(enablePin, HIGH);
  //driver.disable();
  currentState = MotorState::DISABLE;
  axishomed = false;
}

bool StepperMotor::stallStatus() {
  return (bool)((*stallPort & stallBit) != 0);
}


bool StepperMotor::isHomed() {
  return axishomed;
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

FunctionResponse StepperMotor::home(bool execute) {

  unsigned long currentMicros;
  FunctionResponse tResponse;

  switch (homingState) {
    case 0: //idle
    tResponse.busy = false;
    tResponse.done = false;
    tResponse.error = false;
      if ((!prevExecute && execute) && (currentState == MotorState::STANDSTILL)) {
          digitalWrite(dirPin, homingDirection);
          homingState = 10;
          lastMicros = micros();
          pulseCount = 0;
          totalPulseCount = 0;
          axishomed = false;
      }
      break;
    case 10: //homing
      tResponse.busy = true;
      tResponse.done = false;
      tResponse.error = false;
      currentMicros = micros();
      if (currentMicros - lastMicros >= homingStepInterval) {
        lastMicros = currentMicros;
        pulse(20);
        pulseCount++;
        totalPulseCount++;

        if (stallStatus()){
            homingState = 20;
            toggleEnablePin();
        }
        else if (pulseCount >= stepPerMilimeter) { //check each millimeter
          if (totalPulseCount >= maxPulseCount) { //error condition
            homingState = 40;
          }
          pulseCount = 0;
        }
      }
      break;
    case 20: //set position
      tResponse.busy = false;
      tResponse.done = true;
      tResponse.error = false;

      if (!axishomed){

        if (homingDirection){
          currentPosition = limits.maxPosition;
        }
        else {
          currentPosition = limits.minPosition;
        }
        axishomed = true;
      }

      if (!execute) {
        homingState = 10;
      }


      break;
    case 40: //block state
      tResponse.busy = false;
      tResponse.done = false;
      tResponse.error = true;
      homingState = 40; //turn on an LED to indicate the error state
      break;
  }
  prevExecute = execute;
  return tResponse;
  
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
  motorCurrent = current;
}

void StepperMotor::setStallThreshold(int threshold) {
  stallThreshold = threshold;
}

void StepperMotor::toggleEnablePin(){
  digitalWrite(enablePin, HIGH);
  delay(2);
  digitalWrite(enablePin, LOW);
}

MotorState StepperMotor::getState() {
  return currentState;
}