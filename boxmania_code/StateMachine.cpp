// StateMachine.cpp
#include "StateMachine.h"
#include <Arduino.h>
#include <sam.h>

StateMachine::StateMachine(
  StepperConfig& cutterConfig,
  StepperConfig& pusherConfig,
  DcMotorConfig& wheelConfig,
  LedsPins leds,
  SensorsPins sensors
  ): 
  triggerPin(sensors.IR),
  lidPin(sensors.LS),
  wheelSpeed(100), 
  pusher(pusherConfig),
  cutter(cutterConfig),
  wheel(wheelConfig),
  currentState(States::DISABLE),
  redLed(leds.red),
  greenLed(leds.green) {
  //pinMode(redLed, OUTPUT);
  //pinMode(greenLed, OUTPUT);
}

void StateMachine::update() {
  switch (currentState) {
    case States::DISABLE:
      handleDisable();
      break;
    case States::INITIALIZING:
      handleInitializing();
      break;
    case States::IDLE:
      handleIdle();
      break;
    case States::POSITIONING_X:
      handlePositioningX();
      break;
    case States::POSITIONING_Y:
      handlePositioningY();
      break;
    case States::HOLDING:
      handleHolding();
      break;
    case States::CUTTING:
      handleCutting();
      break;
    case States::RELEASING:
      handleReleasing();
      break;
    case States::OPENING_FLAPS:
      handleOpeningFlaps();
      break;
    case States::FLATTENING:
      handleFlattening();
      break;
  }
}

void StateMachine::setState(States newState) {
  currentState = newState;
}


void StateMachine::handleDisable() {
  // set up and configure drivers
  bool setupSuccess = cutter.setupDriver() && pusher.setupDriver();

  if (setupSuccess) {
    Serial.println("Driver setup successful.");
    //configure stepper drivers
    cutter.configDriver();
    pusher.configDriver();
    // disable all actuators (if needed)
    cutter.disable();
    pusher.disable();
    wheel.stop(); 

    setState(States::INITIALIZING);
  } else {
    Serial.println("Driver setup failed. Retrying...");
    //red led flashing
  }
}

void StateMachine::handleInitializing() {
  // Enable stepper drivers
  cutter.enable();
  pusher.enable();

  // Rotate the wheel in reverse for 500ms at 60% spped
  wheel.moveTime(-60, 500); 

  // Home the drivers
  FunctionResponse homeRespCutter;
  FunctionResponse homeRespPusher;
  bool doHomingPusher = false;
  bool doHomingCutter = false;
  bool homingPusherComplete  = false;      
  bool homingCutterComplete  = false;   

  while (!homingPusherComplete && !homingCutterComplete){
    homeRespCutter = cutter.home(doHomingCutter);
    homeRespPusher = pusher.home(doHomingPusher);
    
    if (homeRespCutter.busy){
      doHomingCutter = false;
    }
    else{
      doHomingCutter = true;
    }
    if (homeRespCutter.done){
      homingCutterComplete = true;
      Serial.println("Homing cuter done");
    }
    else if (homeRespCutter.error){
      Serial.println("Homing cutter fail");
      homingCutterComplete = true;
    }

    if (homeRespPusher.busy){
      doHomingPusher = false;
    }
    else{
      doHomingPusher = true;
    }
    if (homeRespPusher.done){
      homingPusherComplete = true;
      Serial.println("Homing cuter done");
    }
    else if (homeRespPusher.error){
      Serial.println("Homing cutter fail");
      homingPusherComplete = true;
    }
  }

  if (cutter.isHomed() && pusher.isHomed()) {
    // Successfully homed both motors
    setState(States::IDLE);
  } else {
    // Homing failed, handle error state
    setState(States::ERROR);
  }
}

void StateMachine::handleIdle() {
  // Check the digital input
  if (analogRead(triggerPin) > 1000) {
    setState(States::POSITIONING_X);
  }
  // If the condition is not met, remain in the IDLE state
}

void StateMachine::handlePositioningX() {
  // Move pusher to position 10mm
  MoveResult moveResult = pusher.moveAbs(10.0);

  // Check if the operation is complete and successful
  if (moveResult.complete) {
    // Transition to the next state
    setState(States::POSITIONING_Y);
  } else {
    // Transition to the error state
    setState(States::ERROR);
  }
}

void StateMachine::handlePositioningY() {
  // Handle the POSITIONING_Y state
  // wheel sequence
}

void StateMachine::handleHolding() {
  // Handle the HOLDING state
  // ...
}

void StateMachine::handleCutting() {
  // Move cutter to position 310.0mm
  MoveResult moveResult = cutter.moveAbs(310.0);

  // Check if the operation is successful
  if (moveResult.complete) {
    // Move cutter back to position 0.0mm
    cutter.moveAbs(0.0);

    // Transition to the next state
    setState(States::OPENING_FLAPS);
  } else {
    // Transition to the error state
    setState(States::ERROR);
  }
}
void StateMachine::handleReleasing() {
  // Handle the RELEASING state
  // ...
}

void StateMachine::handleOpeningFlaps() {
  // Handle the OPENING_FLAPS state
  // ...
}

void StateMachine::handleFlattening() {
  // Move pusher to position 310.0mm
  MoveResult moveResult = pusher.moveAbs(310.0);

  // Check if the operation is successful
  if (pusher.isSetup() && moveResult.complete && !pusher.stallStatus()) {
    // Move pusher back to position 0.0mm
    pusher.moveAbs(0.0);

    // Transition to the next state
    setState(States::INITIALIZING);
  } else {
    // Transition to the error state
    setState(States::ERROR);
  }
}

void StateMachine::handleError() {
  // Handle the ERROR state
  // ...
}
