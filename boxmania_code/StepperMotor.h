#ifndef STEPPERMOTOR_H
#define STEPPERMOTOR_H
#include <Arduino.h>
#include <TMC2209.h>

struct axisLimits {
  float maxPosition;
  float minPosition;
  float maxVelocity;
  float minVelocity;
};

struct homingConfig { 
  //since we are using stall detection for homing, home position must be iqual to maxPosition or minPosition
  bool direction; //0- negative, 1 - positive
  float velocity; 
};

struct StepperConfig {
  int stepPin;
  int dirPin;
  int stallPin;
  int maxSpeed; //mm/s
  int current; //0-100%
  int stallThreshold; //0-200
  axisLimits limits;
  homingConfig homing;
  HardwareSerial& serialPort; //use a diferent serial for each motor
};

enum class MotorState {
  DISABLE,
  STANDSTILL,
  HOMING,
  MOVING
};

class StepperMotor {
public:
  StepperMotor(const StepperConfig& config);

  void moveRelative(float distance);
  void moveAbs(float position);
  void enable();
  void home();
  void disable();
  void setSpeed(int speed);
  void setCurrent(int current);
  void setStallThreshold(int threshold);

  MotorState getState();

private:
  void pulse();
  TMC2209 driver;
  int stallPin;
  int stepPin;
  int dirPin;
  int maxSpeed; //max linear speed in mm/s
  MotorState currentState;
  unsigned long lastStepTime; // Time of the last step
  int speed; // Current speed setting
  float initialSpeed; // Initial speed (10 mm/s in this case)
  float acceleration;  // Constant acceleration (50 mm/s^2 in this case)
  int stepToDistanceFactor;
  uint32_t mmToPulses(float distance);
  float currentPosition; // Current position of the axis
  axisLimits limits;
  bool homingDirection;
  float homingVel;
};

#endif