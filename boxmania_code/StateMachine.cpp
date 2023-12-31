// StateMachine.cpp
#include "StateMachine.h"
#include <Arduino.h>
#include <sam.h>

StateMachine::StateMachine(
    StepperConfig& cutterConfig,
    StepperConfig& pusherConfig,
    DcMotorConfig& wheelConfig,
    int LedPin,
    SensorsPins sensors
  ): 
  triggerPin(sensors.IR),
  lidPin(sensors.LS),
  wheelSpeed(100), 
  pusher(pusherConfig),
  cutter(cutterConfig),
  wheel(wheelConfig),
  currentState(States::DISABLE),
  pixels(NUMPIXELS,LedPin, NEO_GRB + NEO_KHZ800) {
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

  pixels.begin();
  setColor(Color::BLUE,true);
  pinMode(lidPin, INPUT_PULLUP);
  pinMode(triggerPin, INPUT);
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

  // Rotate the wheel in reverse for 200ms at 60% spped
  wheel.moveTime(-60, 100); 

  // Home the drivers
  FunctionResponse homeRespCutter;
  FunctionResponse homeRespPusher;
  bool doHomingPusher = false;
  bool doHomingCutter = false;
  bool homingPusherComplete  = false;      
  bool homingCutterComplete  = false;   
  Serial.println("homing");
  while (!(homingPusherComplete && homingCutterComplete)){
    homeRespCutter = cutter.home(doHomingCutter);
    homeRespPusher = pusher.home(doHomingPusher);
    
    if (homeRespCutter.busy || homingCutterComplete){
      doHomingCutter = false;
    }
    else{
      doHomingCutter = true;
    }

    if (homeRespCutter.done && !homingCutterComplete){
      homingCutterComplete = true;
      Serial.println("Homing cutter done");
    }
    else if (homeRespCutter.error){
      Serial.println("Homing cutter fail");
      homingCutterComplete = true;
    }

    if (homeRespPusher.busy || homingPusherComplete){
      doHomingPusher = false;
    }
    else{
      doHomingPusher = true;
    }

    if (homeRespPusher.done && !homingPusherComplete){
      homingPusherComplete = true;
      Serial.println("Homing pusher done");
    }
    else if (homeRespPusher.error){
      Serial.println("Homing pusher fail");
      homingPusherComplete = true;
    }
  }

  if (cutter.isHomed() && pusher.isHomed()) {
    // Successfully homed both motors
    Serial.println("going to IDLE");
    setState(States::IDLE);
    startTime = 0;
    setColor(Color::GREEN,true);
  } else {
    // Homing failed, handle error state
    Serial.println("going to ERROR");
    setState(States::ERROR);
  }
}

void StateMachine::handleIdle() {

  if (!digitalRead(triggerPin) && !digitalRead(lidPin)) {
    // If the condition is true, start the timer
    if (startTime == 0) {
      startTime = millis();
    } 
    else {
      // Check if the duration has passed
      if (millis() - startTime >= 1000) {
        // Change the state after 1 second
        setState(States::POSITIONING_X);
        startTime = 0;  // Reset the timer
        setColor(Color::RED,true);
      }
    }
  } 
  else {
    // Reset the timer if the condition is not true
    startTime = 0;
  }

}

void StateMachine::handlePositioningX() {
  // Move pusher to position 10mm
  MoveResult moveResult = pusher.moveAbs(65.0);

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
  // wheel sequence if pushing is required
  setState(States::CUTTING);
}

void StateMachine::handleHolding() {
  // Handle the HOLDING state
  // ...
}

void StateMachine::handleCutting() {
  // Move cutter to position 310.0mm
  MoveResult moveResult = cutter.moveAbs(375.0);

  // Check if the operation is successful
  if (moveResult.complete) {
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
  wheel.moveTime(50, 100);
  wheel.moveTime(60, 100);
  wheel.moveTime(70, 100);
  wheel.moveTime(80, 100);
  wheel.moveTime(90, 100);
  wheel.moveTime(100, 3000);
  delay(200); 
  wheel.moveTime(-100, 500);
  setState(States::FLATTENING);
}

void StateMachine::handleFlattening() {
  // Move pusher to position 310.0mm
  MoveResult moveResult = pusher.moveAbs(340.0);

  // Check if the operation is successful
  if (pusher.isSetup() && moveResult.complete && !pusher.stallStatus()) {
    // Move pusher back to position 0.0mm
    pusher.moveAbs(50.0);
    cutter.moveAbs(1.0);

    // Transition to the next state
    setState(States::IDLE);
    setColor(Color::GREEN,true);
  } else {
    // Transition to the error state
    Serial.println("fail");
    setState(States::ERROR);
  }
}

void StateMachine::handleError() {
  setColor(Color::RED,false);
  delay(500);
  setColor(Color::BLUE,false);
  delay(500);
}

void StateMachine::setColor(Color color, bool cool) {
  uint8_t r = 0;
  uint8_t g = 0;
  uint8_t b = 0;
  switch (color) {
    case Color::RED:
      r = 255;
      g = 0;
      b = 0;
      break;
    case Color::GREEN:
      r = 0;
      g = 255;
      b = 0;
      break;
    case Color::BLUE:
      r = 0;
      g = 255;
      b = 255;
      break;
    // Add more cases for other colors as needed
  }

  pixels.clear();
  if (cool){
    for(int i=0; i<NUMPIXELS; i++) { // For each pixel...
      pixels.setPixelColor(i, pixels.Color(r, g, b));
      pixels.show();   // Send the updated pixel colors to the hardware.
      delay(10); // Pause before next pass through loop
    }
  }
  else{
    pixels.fill(pixels.Color(r, g, b));
  }

}

