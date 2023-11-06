// StateMachine.cpp
#include "StateMachine.h"

StateMachine::StateMachine(
  StepperConfig& cutterConfig,
  StepperConfig& pusherConfig,
  DcMotorConfig& wheelConfig
  ) : 
  cutter(cutterConfig), 
  pusher(pusherConfig), 
  wheel(wheelConfig) {
  currentState = States::DISABLE; // Initialize with DISABLE state
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
  // Handle the DISABLE state
  // ...
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
