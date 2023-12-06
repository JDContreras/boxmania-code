#include "StateMachine.h"
#include "StepperMotor.h"
#include <Arduino.h>   // required before wiring_private.h
#include "wiring_private.h" // pinPeripheral() function
#include <TMC2209.h>


StepperConfig cutterMotor = {
    .stepPin = SCK,                
    .dirPin = MOSI,
    .stallPin = A5,
    .enPin = MISO,
    .current = 60, //60-40/30
    .stallThreshold = 40,
    .stepPerMilimeter = 40,
    .microsteps = 8,
    .holdCurrent = 40,
    .limits = {    //mm              
        .maxPosition = 311.0,
        .minPosition = 0.0,
        .maxVelocity = 50.0,
        .minVelocity = 10.0
    },
    .homing = {                 
        .direction = false,       // true for positive direction, false for negative
        .velocity = 20.0
    },
    .serialPort = Serial1,
    .address = 1
};

StepperConfig pusherMotor = {
    .stepPin = 6,                
    .dirPin = 5,
    .stallPin = 9,
    .enPin = 10,
    .current = 50,
    .stallThreshold = 20,
    .stepPerMilimeter = 40,
    .microsteps = 8,
    .holdCurrent = 40,
    .limits = {    //mm              
        .maxPosition = 310.0,
        .minPosition = 0.0,
        .maxVelocity = 50.0,
        .minVelocity = 10.0
    },
    .homing = {                 // Initialize the nested struct
        .direction = false,       // true for positive direction, false for negative
        .velocity = 20.0
    },
    .serialPort = Serial1,
    .address = 3
};

DcMotorConfig wheelMotor = {
  .forwardPin = A3,      
  .reversePin = A4,     
  .maxSpeed = 100      
};

LedsPins leds = {
  .red = 13,
  .green = LED_BUILTIN 
};

SensorsPins sensors = {
  .IR = A2,
  .LS = A1 
};

//StateMachine stateMachine(cutterMotor,pusherMotor,wheelMotor);

StateMachine stateMachine {
  .cutterConfig = cutterMotor,
  .pusherConfig = pusherMotor,
  .wheelConfig = wheelMotor,
  .leds = leds,
  .sensors = sensors
};


void setup() {
  Serial.begin(1000000);
}

void loop() {
  stateMachine.update();
}