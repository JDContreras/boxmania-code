#include "StateMachine.h"
#include "StepperMotor.h"
#include <Arduino.h>   // required before wiring_private.h
#include "wiring_private.h" // pinPeripheral() function
#include <TMC2209.h>

//TMC2209 cutter_driver;
//TMC2209 pusher_driver;
Uart Serial2 (&sercom1, 11, 10, SERCOM_RX_PAD_0, UART_TX_PAD_2);

void SERCOM1_Handler()
{
  Serial2.IrqHandler();
}


StepperConfig cutterMotor = {
    .stepPin = MISO,                
    .dirPin = MOSI,
    .stallPin = A5,
    .enPin = SCK,
    .current = 80, //60
    .stallThreshold = 40,
    .stepPerMilimeter = 40,
    .limits = {    //mm              
        .maxPosition = 311.0,
        .minPosition = 0.0,
        .maxVelocity = 50.0,
        .minVelocity = 10.0
    },
    .homing = {                 // Initialize the nested struct
        .direction = false,       // true for positive direction, false for negative
        .velocity = 20.0
    },
    //.driver = cutter_driver,
    .serialPort = Serial1      // Initialize with a specific serial port
};

StepperConfig pusherMotor = {
    .stepPin = 6,                
    .dirPin = 5,
    .stallPin = 9,
    .enPin = 12,
    .current = 50,
    .stallThreshold = 20,
    .stepPerMilimeter = 40,
    .limits = {    //mm              
        .maxPosition = 200.0,
        .minPosition = 0.0,
        .maxVelocity = 50.0,
        .minVelocity = 10.0
    },
    .homing = {                 // Initialize the nested struct
        .direction = false,       // true for positive direction, false for negative
        .velocity = 20.0
    },
    //.driver = pusher_driver,
    .serialPort = Serial2      // Initialize with a specific serial port
};

DcMotorConfig wheelMotor = {
  .forwardPin = A3,      
  .reversePin = A4,     
  .maxSpeed = 100      
};

Leds leds = {
  .red = 13,
  .green = LED_BUILTIN 
};

//StateMachine stateMachine(cutterMotor,pusherMotor,wheelMotor);

StateMachine stateMachine {
  .cutterConfig = cutterMotor,
  .pusherConfig = pusherMotor,
  .wheelConfig = wheelMotor,
  .leds = leds
};


void setup() {
  Serial.begin(1000000);
  pinPeripheral(10, PIO_SERCOM);
  pinPeripheral(11, PIO_SERCOM);
  pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
  stateMachine.update();
}