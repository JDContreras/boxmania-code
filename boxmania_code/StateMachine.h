// StateMachine.h
#ifndef STATEMACHINE_H
#define STATEMACHINE_H
#define NUMPIXELS 100

#include "StepperMotor.h"
#include "DCMotor.h"
#define DEBUG
#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
 #include <avr/power.h> // Required for 16 MHz Adafruit Trinket
#endif

struct LedsPins {
  int red;
  int green;
};

struct SensorsPins {
  int IR;
  int LS;
};

enum class Color {
  RED,
  GREEN,
  BLUE,
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
      int LedPin,
      SensorsPins sensors
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
    void setColor(
      Color color,
      bool cool
    );
    StepperMotor cutter;
    StepperMotor pusher;
    int wheelSpeed;
    DCMotor wheel; 
    int triggerPin;
    int lidPin;
    bool doHoming;
    Adafruit_NeoPixel pixels;
    unsigned long startTime = 0;  
};

#endif