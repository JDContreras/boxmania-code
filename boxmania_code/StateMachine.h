// StateMachine.h
#ifndef STATEMACHINE_H
#define STATEMACHINE_H
#include <Arduino.h>

enum class States {
  DISABLE,
  INITIALIZING,
  IDLE,
  POSITIONING_X,
  POSITIONING_Y,
  HOLDING,
  CUTTING,
  RELEASING, // rekeasing state
  OPENING_FLAPS,
  FLATTENING,
  ERROR
};


class StateMachine {
public:
  StateMachine();  // Constructor
  void update();   // Main update function

private:
  States currentState;
  void setState(States newState);
  // State-specific methods
  void handleDisable();
  void handleInitializing();
  void handleIdle();
  void handlePositioningX();
  void handlePositioningY();
  void handleHolding();
  void handleCutting();
  void handleReleasing();
  void handleOpeningFlaps();
  void handleFlattening();
  void handleError();
  public:
  // Public method to access the current state if needed
  States getCurrentState() {
    return currentState;
  }
};

#endif