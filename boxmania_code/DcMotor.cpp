#include "DCMotor.h"
#include <Arduino.h>

DCMotor::DCMotor(DcMotorConfig& config)
  : forwardPin(config.forwardPin), reversePin(config.reversePin), prevSpeed(0) {
  maxSpeed = constrain(config.maxSpeed, 0, 100); //speed limit to 100
  pinMode(forwardPin, OUTPUT);
  pinMode(reversePin, OUTPUT);

}

void DCMotor::run(int speed) {
  // Constrain the speed to the range -maxSpeed to maxSpeed
  speed = constrain(speed, -maxSpeed, maxSpeed);
  //check if there is a change in direction

  if (speed * prevSpeed < 0) {
    // Change in direction, stop the motor for 200ms
    analogWrite(forwardPin, 0);
    analogWrite(reversePin, 0);
    digitalWrite(reversePin, LOW);
    digitalWrite(forwardPin, LOW);
    delay(200);
  }

  if (speed > 0) {
    analogWrite(reversePin, 0);
    digitalWrite(reversePin, LOW);
    analogWrite(forwardPin, map(speed, 0, 100, 0, 255));
    
  } else if (speed < 0) {
    analogWrite(forwardPin, 0);
    digitalWrite(forwardPin, LOW);
    analogWrite(reversePin, map(abs(speed), 0, 100, 0, 255));
  } else {
    digitalWrite(forwardPin, LOW);
    digitalWrite(reversePin, LOW);
    analogWrite(forwardPin, 0);
    analogWrite(reversePin, 0);
  }
  prevSpeed = speed;
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