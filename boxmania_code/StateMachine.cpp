// StateMachine.cpp
#include "StateMachine.h"
#include <Arduino.h>

StateMachine::StateMachine(
  StepperConfig& cutterConfig,
  StepperConfig& pusherConfig,
  DcMotorConfig& wheelConfig,
  Leds& leds
  ) : 

  cutter(cutterConfig), 
  pusher(pusherConfig),
  wheel(wheelConfig),
  currentState(States::DISABLE),
  redLed(leds.red),
  greenLed(leds.green) {
  pinMode(redLed, OUTPUT);
  pinMode(greenLed, OUTPUT);
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

// Implement the state-specific methods as needed
void StateMachine::handleDisable() {
  /*
  // Handle the DISABLE state
  cutter.enable();
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');  // Read the incoming command
    input.trim();  // Remove leading/trailing whitespace
    bool complete = false;
    FunctionResponse homeResp;
    int distance;
    switch (input[0]) {
      case 'I':
        Serial.println("Enabling axis");
        cutter.enable();
      break;

      case 'H':
        // Homing operation
        #ifdef DEBUG
        Serial.println("Homing operation started");
        #endif
        
        doHoming = true;
        while (!complete){
          homeResp = cutter.home(doHoming);
          if (homeResp.busy){
            doHoming = false;
          }
          else if (homeResp.done){
            complete = true;
          }
          else if (homeResp.error){
            #ifdef DEBUG
              Serial.println("Homing fail");
              complete = true;
            #endif
          }
        
        }

        break;

      case 'M':
        distance = input.substring(2).toInt();
        if (distance >= 0 && distance <= 100) {
          // Move to the specified distance
          Serial.print("Moving to position ");
          Serial.print(distance);
          Serial.println("mm");
          // Replace this with your actual stepper motor movement logic
        } else {
          Serial.println("Invalid distance. Please use a value between 0 and 100.");
        }
        break;

      default:
        Serial.println("Unknown command. Valid commands: 'H' for homing, 'M X' for moving to position X mm.");
    }
  }
  */
}
void StateMachine::handleInitializing() {
  // Handle the INITIALIZING state
  // ...
}

void StateMachine::handleIdle() {
  // Handle the IDLE state
  // ...
}

void StateMachine::handlePositioningX() {
  // Handle the POSITIONING_X state
  // ...
}

void StateMachine::handlePositioningY() {
  // Handle the POSITIONING_Y state
  // ...
}

void StateMachine::handleHolding() {
  // Handle the HOLDING state
  // ...
}

void StateMachine::handleCutting() {
  // Handle the CUTTING state
  // ...
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
  // Handle the FLATTENING state
  // ...
}

void StateMachine::handleError() {
  // Handle the ERROR state
  // ...
}
