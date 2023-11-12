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
  wheelSpeed(100), 
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
  delay(1000);
  Serial.println("Configuring driver");
  cutter.configDriver();
  setState(States::INITIALIZING);
}
void StateMachine::handleInitializing() {
  // Handle the INITIALIZING state
  cutter.enable();
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');  // Read the incoming command
    input.trim();  // Remove leading/trailing whitespace
    bool complete = false;
    FunctionResponse homeResp;
    int distance;
    int speed;
    int count = 0;
    switch (input[0]) {
      case 'E':
        Serial.println("Enabling axis");
        //cutter.configDriver();
        cutter.enable();
      break;

      case 'C':
        Serial.println("Configuring axis");
        //cutter.configDriver();
        cutter.configDriver();
      break;

      case 'D':
        Serial.println("Disabling axis");
        //cutter.configDriver();
        cutter.disable();
      break;

      case 'S':
        speed = input.substring(2).toInt();
        Serial.println("setting speed");
        cutter.setSpeed(speed);
      break;

      case 'H':
        // Homing operation
        #ifdef DEBUG
        Serial.println("Homing operation started");
        #endif
        
        doHoming = false;
        
        while (!complete){
          homeResp = cutter.home(doHoming);
          
          if (homeResp.busy){
            doHoming = false;
          }
          else{
            doHoming = true;
          }
          if (homeResp.done){
            complete = true;
          }
          else if (homeResp.error){
            #ifdef DEBUG
              Serial.println("Homing fail");
            #endif
            complete = true;
          }
          /*
          count++;
          if (count>400){ //currentSpeed
            #ifdef DEBUG
              Serial.print("Busy ");
              Serial.println(homeResp.busy);
              Serial.print("Done ");
              Serial.println(homeResp.done);
              Serial.print("Error ");
              Serial.println(homeResp.error);
            #endif
          }
          */
        }
        
        break;

      case 'M':
        distance = input.substring(2).toInt();
        if (distance >= -300 && distance <= 300) {
          // Move to the specified distance
          Serial.print("Moving to position ");
          Serial.print(distance);
          Serial.println("mm");
          cutter.moveRelative(distance);
        } 
        else {
          Serial.println("Invalid distance. Please use a value between 0 and 100.");
        }
        break;

      case 'N':
        if (cutter.isHomed()){
          setState(States::IDLE);
          Serial.println("Going to IDLE State");
        }
        else{
            Serial.println("home the axis first");
        }
        break;


      default:
        Serial.println("Unknown command. Valid commands: 'H' for homing, 'M X' for moving to position X mm, 'F' to change to IDLE");
    }
  }
  
}

void StateMachine::handleIdle() {
  // Handle the IDLE state
  // waiting for the box
  //int time; 
  if (Serial.available() > 0) {
    unsigned long time;
    int tempSpeed;
    String input = Serial.readStringUntil('\n');  // Read the incoming command
    input.trim();  // Remove leading/trailing whitespace
    switch (input[0]) {
      case 'F':
        Serial.println("Moving forward");
        cutter.moveAbs(310);
      break;


      case 'R':
        cutter.moveAbs(0);
        Serial.println("Moving reverse");

      break;

      case 'W':
        Serial.print("Position ");
        Serial.print(cutter.getCurrentPosition());
        Serial.println(" mm");
      break;

      case 'Y':
        time = input.substring(2).toInt();
        if (time > 0 && time <= 5000) {
          // Move to the specified distance
          Serial.print("Moving wheel ");
          Serial.print(time);
          Serial.println(" ms");
          wheel.moveTime(wheelSpeed, time);
        } 
        else {
          Serial.println("Invalid time. Please use a value between 0 and 5000.");
        }
      break;

      case 'Z':
          time = input.substring(2).toInt();
          if (time > 0 && time <= 5000) {
          // Move to the specified distance
          Serial.print("Moving wheel ");
          Serial.print(time);
          Serial.println(" ms");
          wheel.moveTime(-wheelSpeed, time);
        }  
        else {
          Serial.println("Invalid time. Please use a value between 0 and 5000.");
        }
      break;

      case 'S':
          tempSpeed = input.substring(2).toInt();
          if (tempSpeed >= 20 && tempSpeed <= 100) {
          // Move to the specified distance
          Serial.print("Moving wheel ");
          Serial.print(time);
          Serial.println(" ms");
          wheelSpeed = tempSpeed;
        }  
        else {
          Serial.println("Invalid time. Please use a value between 20 and 100.");
        }
      break;

      case 'N':
        setState(States::IDLE);
        Serial.println("Going to DISABLE State");
      break;


    }
  }
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
