#ifndef STEPPERMOTOR_H
#define STEPPERMOTOR_H
#include <Arduino.h>
#include <TMC2209.h>
#include "DataTypes.h"



struct AxisLimits {
  float maxPosition;
  float minPosition;
  float maxVelocity;
  float minVelocity;
};

struct HomingConfig { 
  //since we are using stall detection for homing, home position must be iqual to maxPosition or minPosition
  bool direction; //false- negative, true - positive
  float velocity; 
};

struct StepperConfig {
  int stepPin;
  int dirPin;
  int stallPin;
  int enPin;
  int current; //0-100%
  int stallThreshold; //0-200
  int stepPerMilimeter; //steps per mm, depent of the pulley or gearbox
  uint16_t microsteps;
  uint8_t holdCurrent;
  AxisLimits limits;
  HomingConfig homing;
  HardwareSerial& serialPort; 
  uint8_t address;
};

struct MoveResult {
  float distance = 0.0;
  bool complete = false;
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
  MoveResult moveRelative(float distance);
  void moveAbs(float position);
  void enable();
  FunctionResponse home(bool execute);
  bool isHomed();
  void disable();
  void setSpeed(int speed);
  void setCurrent(int current);
  void setStallThreshold(int threshold);
  bool stallStatus(); //read stallPin to check if the stall is detected. 
  MotorState getState();
  bool configDriver();
  bool setupDriver();
  void checkDriver();
  float getCurrentPosition();
  void printConfig();
  void printStatus();
private:
  static const int BAUD_RATE = 250000;
  void toggleEnablePin();
  void pulse(int stepInterval);
  TMC2209 driver;
  int stepPin;
  int dirPin;
  int enablePin;
  int maxSpeed; //max linear speed in mm/s
  MotorState currentState;
  unsigned long lastStepTime; // Time of the last step
  int speed; // Current speed setting
  float initialSpeed; // Initial speed (10 mm/s in this case)
  float acceleration;  // Constant acceleration (50 mm/s^2 in this case)
  int stepToDistanceFactor;
  uint32_t mmToPulses(float distance);
  float currentPosition; // Current position of the axis
  AxisLimits limits;
  bool homingDirection;
  int homingStepInterval;
  int stepPerMilimeter;
  // Define the port and bit number of the stallPin
  volatile uint32_t* stallPort;
  uint32_t stallBit;
  bool axishomed; //flag indicating axis has been homed
  int homingState;
  unsigned long lastMicros;
  int maxPulseCount;
  int pulseCount;
  int totalPulseCount;
  bool prevExecute ;
  uint8_t motorCurrent;
  uint8_t stallThreshold;
  HardwareSerial& serialPort;
  uint8_t address;
  uint16_t microsteps;
  uint8_t holdCurrent;
};

#endif