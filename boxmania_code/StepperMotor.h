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
  int current; //0-100%
  int stallThreshold; //0-200
  int stepPerMilimeter; //steps per mm, depent of the pulley or gearbox
  axisLimits limits;
  homingConfig homing;
  HardwareSerial& serialPort; //use a diferent serial for each motor
};

struct moveResult {
  float distance;
  bool complete;
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
  moveResult moveRelative(float distance);
  void moveAbs(float position);
  void enable();
  void home();
  void disable();
  void setSpeed(int speed);
  void setCurrent(int current);
  void setStallThreshold(int threshold);
  bool stallStatus(); //read stallPin to check if the stall is detected. 
  MotorState getState();

private:
  void pulse(int stepInterval);
  TMC2209 driver;
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
  int stepPerMilimeter;
  // Define the port and bit number of the stallPin
  volatile uint32_t* stallPort;
  uint32_t stallBit;
  bool axishomed; //flag indicating axis has been homed
};

#endif