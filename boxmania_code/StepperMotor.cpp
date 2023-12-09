#include "StepperMotor.h"

StepperMotor::StepperMotor(const StepperConfig& config)
  : stepPin(config.stepPin), 
    dirPin(config.dirPin), 
    enablePin(config.enPin),
    speed(config.limits.maxVelocity),
    holdCurrent(config.holdCurrent),
    microsteps(config.microsteps),
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
    serialPort(config.serialPort),
    address(config.address)
  {
  pinMode(config.stepPin, OUTPUT);
  pinMode(config.enPin, OUTPUT);
  pinMode(config.dirPin, OUTPUT);
  pinMode(config.stallPin, INPUT);
}

bool StepperMotor::isSetup(){
  return driver.isSetupAndCommunicating();
}

void StepperMotor::checkDriver(){
  Serial.print("isCommunicating: ");
  Serial.println(driver.isCommunicating());
  Serial.print("isSetupAndCommunicating: ");
  Serial.println(driver.isSetupAndCommunicating());
  Serial.print("isCommunicatingButNotSetup: ");
  Serial.println(driver.isCommunicatingButNotSetup());
}

bool StepperMotor::setupDriver(){
  Serial.print("setting up driver with address: ");
  Serial.println(address);
  switch (address) {
    case 0:
      driver.setup(serialPort, BAUD_RATE,TMC2209::SERIAL_ADDRESS_0);
      break;
    case 1:
      driver.setup(serialPort, BAUD_RATE,TMC2209::SERIAL_ADDRESS_1);
      break;
    case 2:
      driver.setup(serialPort, BAUD_RATE,TMC2209::SERIAL_ADDRESS_2);
      break;
    case 3:
      driver.setup(serialPort, BAUD_RATE,TMC2209::SERIAL_ADDRESS_3);
      break;
    default:
      Serial.println("Invalid address");
      return false;
  }
  driver.setReplyDelay(4);
  delay(400);
  driver.disableStealthChop();
  if (driver.isSetupAndCommunicating()){
    return true;
  }
  else {
    return false;
  }
}


bool StepperMotor::configDriver(){
  Serial.print("configuring driver with address: ");
  Serial.println(address);
  if (driver.isSetupAndCommunicating()){
    Serial.println("Setting up driver parameters...");
    driver.setHardwareEnablePin(enablePin);
    driver.disableCoolStep(); //necesary to use stall detection
    driver.disableStealthChop();
    driver.setCoolStepDurationThreshold(1000000);
    driver.setStealthChopDurationThreshold(10);
    delay(100);

    driver.enableStealthChop();
    driver.enableAutomaticCurrentScaling();

    Serial.print("Setting stallGuardThreshold: ");
    Serial.println(stallThreshold);
    driver.setStallGuardThreshold(stallThreshold);

    Serial.print("Setting runCurrent: ");
    Serial.println(motorCurrent);
    driver.setRunCurrent(motorCurrent);

    driver.setStandstillMode(driver.NORMAL);

    Serial.print("Setting HoldCurrent: ");
    Serial.println(holdCurrent);
    driver.setHoldCurrent(holdCurrent);

    Serial.print("Setting MicrostepsPerStep: ");
    Serial.println(microsteps);
    driver.setMicrostepsPerStep(microsteps); //TODO ad argument for this

    driver.enable();
    Serial.println("driver configured");
    return true;
  }
  else {
    Serial.println("driver no conected or set up");
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

  Serial.print("total steps   ");
  Serial.println(totalSteps);
  Serial.print("accel steps   ");
  Serial.println(accelerationSteps);
  Serial.print("diaccel steps   ");
  Serial.println(accelerationSteps + constantSpeedSteps);

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
  driver.setStallGuardThreshold(stallThreshold-10);
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
  //digitalWrite(enablePin, LOW);
  currentState = MotorState::STANDSTILL;
}

void StepperMotor::disable() {
  // disable implementation
  //digitalWrite(enablePin, HIGH);
  driver.disable();
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
  PORT->Group[g_APinDescription[13].ulPort].OUTSET.reg = (1ul << g_APinDescription[13].ulPin);
  // Wait for 10 microseconds (you can adjust the delay to match your requirements)
  delayMicroseconds(10);
  // Set stepPin low
  PORT->Group[g_APinDescription[stepPin].ulPort].OUTCLR.reg = (1ul << g_APinDescription[stepPin].ulPin);
  PORT->Group[g_APinDescription[13].ulPort].OUTCLR.reg = (1ul << g_APinDescription[13].ulPin);
  delayMicroseconds(stepInterval-10);
}

FunctionResponse StepperMotor::home(bool execute) {
  driver.setStallGuardThreshold(stallThreshold);
  unsigned long currentMicros;
  FunctionResponse tResponse;

  switch (homingState) {
    case 0: //idle
    tResponse.busy = false;
    tResponse.done = false;
    tResponse.error = false;
      if ((!prevExecute && execute) && (currentState == MotorState::STANDSTILL)) {
          //digitalWrite(dirPin, homingDirection);
          homingState = 10;
          lastMicros = micros();
          pulseCount = 0;
          totalPulseCount = 0;
          axishomed = false;
      }
      break;
    case 10: //homing
      digitalWrite(dirPin, homingDirection);
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

void StepperMotor::printConfig(){
  TMC2209::Settings settings = driver.getSettings();

  if (settings.is_communicating) {
    Serial.println("Stepper Driver Configuration:");
    Serial.println("Setup: " + String(settings.is_setup));
    Serial.println("Software Enabled: " + String(settings.software_enabled));
    Serial.println("Microsteps per Step: " + String(settings.microsteps_per_step));
    Serial.println("Inverse Motor Direction Enabled: " + String(settings.inverse_motor_direction_enabled));
    Serial.println("StealthChop Enabled: " + String(settings.stealth_chop_enabled));
    Serial.println("Standstill Mode: " + String(settings.standstill_mode));
    Serial.println("IRun Percentage: " + String(settings.irun_percent));
    Serial.println("IRun Register Value: " + String(settings.irun_register_value));
    Serial.println("IHold Percentage: " + String(settings.ihold_percent));
    Serial.println("IHold Register Value: " + String(settings.ihold_register_value));
    Serial.println("IHoldDelay Percentage: " + String(settings.iholddelay_percent));
    Serial.println("IHoldDelay Register Value: " + String(settings.iholddelay_register_value));
    Serial.println("Automatic Current Scaling Enabled: " + String(settings.automatic_current_scaling_enabled));
    Serial.println("Automatic Gradient Adaptation Enabled: " + String(settings.automatic_gradient_adaptation_enabled));
    Serial.println("PWM Offset: " + String(settings.pwm_offset));
    Serial.println("PWM Gradient: " + String(settings.pwm_gradient));
    Serial.println("CoolStep Enabled: " + String(settings.cool_step_enabled));
    Serial.println("Analog Current Scaling Enabled: " + String(settings.analog_current_scaling_enabled));
    Serial.println("Internal Sense Resistors Enabled: " + String(settings.internal_sense_resistors_enabled));
  } else {
    Serial.println("Communication: Failed");
  }
}

void StepperMotor::printStatus() {
  TMC2209::Status status = driver.getStatus();

  Serial.println("Stepper Driver Status:");

  Serial.println("Over Temperature Warning: " + String(status.over_temperature_warning));
  Serial.println("Over Temperature Shutdown: " + String(status.over_temperature_shutdown));
  Serial.println("Short to Ground A: " + String(status.short_to_ground_a));
  Serial.println("Short to Ground B: " + String(status.short_to_ground_b));
  Serial.println("Low Side Short A: " + String(status.low_side_short_a));
  Serial.println("Low Side Short B: " + String(status.low_side_short_b));
  Serial.println("Open Load A: " + String(status.open_load_a));
  Serial.println("Open Load B: " + String(status.open_load_b));
  Serial.println("Over Temperature 120°C: " + String(status.over_temperature_120c));
  Serial.println("Over Temperature 143°C: " + String(status.over_temperature_143c));
  Serial.println("Over Temperature 150°C: " + String(status.over_temperature_150c));
  Serial.println("Over Temperature 157°C: " + String(status.over_temperature_157c));
  Serial.println("Current Scaling: " + String(status.current_scaling));
  Serial.println("StealthChop Mode: " + String(status.stealth_chop_mode));
  Serial.println("Standstill: " + String(status.standstill));
}