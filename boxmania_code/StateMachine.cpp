// StateMachine.cpp
#include "StateMachine.h"
#include <Arduino.h>
#include <sam.h>

StateMachine::StateMachine(
  StepperConfig& cutterConfig,
  StepperConfig& pusherConfig,
  DcMotorConfig& wheelConfig,
  Leds& leds
  ): 

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
  //set up driver
  //configure drivers
  //disable all actuators
  //red led flashing 
  setState(States::INITIALIZING);
}

void StateMachine::handleInitializing() {
  // Handle the INITIALIZING state
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');  // Read the incoming command
    input.trim();  // Remove leading/trailing whitespace
    bool complete = false;
    FunctionResponse homeResp;
    int distance;
    int speed;
    int count = 0;
    int threshold;
    switch (input[0]) {
      case 'X':
        if (cutter.setupDriver()){
          Serial.println("set up successful ");
        }
        else{
          Serial.println("set up fail");
        }
      break;

      case 'x':
        if (pusher.setupDriver()){
          Serial.println("set up successful ");
        }
        else{
          Serial.println("set up fail");
        }
      break;

      case 'J':
        cutter.checkDriver();
      break;

      case 'j':
        pusher.checkDriver();
      break;

      case 'E':
        Serial.println("Enabling cutter");
        cutter.enable();
      break;

      case 'e':
        Serial.println("Enabling pusher");
        pusher.enable();
      break;

      case 'A':
        cutter.printConfig();
      break;

      case 'a':
        pusher.printConfig();
      break;

      case 'C':
        cutter.configDriver();
      break;

      case 'c':
        pusher.configDriver();
      break;

      case 'D':
        Serial.println("Disabling cutter");
        cutter.disable();
      break;

      case 'd':
        Serial.println("Disabling pusher");
        pusher.disable();
      break;

      case 'H':
        // Homing operation
        Serial.println("Cutter homing operation started");  
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
              Serial.println("Homing fail");
            complete = true;
          }
        }
        complete = false;
        break;
      
      case 'h':
        // Homing operation
        Serial.println("Pusher homing operation started");  
        doHoming = false;
        
        while (!complete){
          homeResp = pusher.home(doHoming);
          
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
              Serial.println("Homing fail");
            complete = true;
          }
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

      case 'm':
        distance = input.substring(2).toInt();
        if (distance >= -300 && distance <= 300) {
          // Move to the specified distance
          Serial.print("Moving to position ");
          Serial.print(distance);
          Serial.println("mm");
          pusher.moveRelative(distance);
        } 
        else {
          Serial.println("Invalid distance. Please use a value between 0 and 100.");
        }
        break;


      case 'W':   //go to wheel control
        setState(States::POSITIONING_X);
        Serial.println("Going to Pos X State");

        break;

      case 'N':     //go to cutter control
      Serial.println("Going to IDLE State");
        setState(States::IDLE);
        Serial.println("Going to IDLE State");
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

      case 'f':
        Serial.println("Moving forward");
        pusher.moveAbs(297);
      break;

      case 'r':
        Serial.println("Moving reverse");
        pusher.moveAbs(0);
      break;

      case 'N':  //go to initial state
        setState(States::INITIALIZING);
        Serial.println("Going to Initializing State");
      break;

      default:
        Serial.println("Unknown command");

    }
  }
}

void StateMachine::handlePositioningX() {
  // Handle the POSITIONING_X state
  if (Serial.available() > 0) {
    unsigned long time;
    int tempSpeed;
    String input = Serial.readStringUntil('\n');  // Read the incoming command
    input.trim();  // Remove leading/trailing whitespace
    switch (input[0]) {
      case 'F':
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

      case 'R':
          time = input.substring(2).toInt();
          if (time > 30 && time <= 5000) {
          // Move to the specified distance
          Serial.print("Moving wheel ");
          Serial.print(time);
          Serial.println(" ms");
          wheel.moveTime(-wheelSpeed, time);
        }  
        else {
          Serial.println("Invalid time. Please use a value between 30 and 5000.");
        }
      break;

      case 's':
          tempSpeed = input.substring(2).toInt();
          if (tempSpeed >= 20 && tempSpeed <= 100) {
          // Move to the specified distance
          Serial.println("Updating speed");
          wheelSpeed = tempSpeed;
          Serial.println("Done");
        }  
        else {
          Serial.println("Invalid spped. Please use a value between 20 and 100.");
        }
      break;

      case 'N':  //go to initial state
        setState(States::INITIALIZING);
        Serial.println("Going to Initializing State");
      break;

      default:
        Serial.println("Unknown command");
    }
  }
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
