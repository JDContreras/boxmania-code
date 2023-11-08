// StateMachine.h
#ifndef STATEMACHINE_H
#define STATEMACHINE_H

#include "StepperMotor.h"
#include "DCMotor.h"
#define DEBUG
struct Leds {
  int red;
  int green;
};

enum class States {
  DISABLE,
  INITIALIZING,
  IDLE,
  POSITIONING_X,
  POSITIONING_Y,
  HOLDING,
  CUTTING,
  RELEASING,
  OPENING_FLAPS,
  FLATTENING,
  ERROR
};


class StateMachine {
  public:
    StateMachine(
      StepperConfig& cutterConfig,
      StepperConfig& pusherConfig,
      DcMotorConfig& wheelConfig,
      Leds& leds
    );  // Constructor

    void update();   // Main update function
    States getCurrentState() {
      return currentState;
    }
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
    StepperMotor cutter;
    StepperMotor pusher;
    DCMotor wheel; 
    int redLed;
    int greenLed;
    bool doHoming;
};

#endif