#include "DCMotor.h"
#include <Arduino.h>

DCMotor::DCMotor(DcMotorConfig& config)
  : forwardPin(config.forwardPin), reversePin(config.reversePin), maxSpeed(config.maxSpeed) {
  pinMode(forwardPin, OUTPUT);
  pinMode(reversePin, OUTPUT);
}

void DCMotor::run(int speed) {
  // Constrain the speed to the range -100 to 100
  speed = constrain(speed, -100, 100);
  delay(200); //wait to stop when changing the direction
  if (speed > 0) {
    analogWrite(forwardPin, map(speed, 0, 100, 0, 255));
    digitalWrite(reversePin, LOW);
  } else if (speed < 0) {
    analogWrite(reversePin, map(abs(speed), 0, 100, 0, 255));
    digitalWrite(forwardPin, LOW);
  } else {
    digitalWrite(forwardPin, LOW);
    digitalWrite(reversePin, LOW);
  }
}

void DCMotor::stop() {
  run(0); // Equivalent to setting speed to 0
}