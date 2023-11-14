#include "DCMotor.h"
#include <Arduino.h>

DCMotor::DCMotor(DcMotorConfig& config)
  : forwardPin(config.forwardPin), reversePin(config.reversePin) {
  maxSpeed = constrain(config.maxSpeed, 0, 100); //speed limit to 100
  pinMode(forwardPin, OUTPUT);
  pinMode(reversePin, OUTPUT);

}

void DCMotor::run(int speed) {
  // Constrain the speed to the range -maxSpeed to maxSpeed
  speed = constrain(speed, -maxSpeed, maxSpeed);
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
    analogWrite(forwardPin, 0);
    analogWrite(reversePin, 0);
  }
}

void DCMotor::stop() {
  run(0); // Equivalent to setting speed to 0
}

void DCMotor::moveTime(int speed, unsigned long time) {
  Serial.println("Runing Wheel");
  run(speed);   // Start the motor with the given speed
  delay(time);  // Delay for the specified duration
  stop();     // Stop the motor after the delay
  Serial.println("Stop Wheel");
}