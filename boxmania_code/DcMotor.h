#ifndef DCMOTOR_H
#define DCMOTOR_H

struct DcMotorConfig {
  int forwardPin;
  int reversePin;
  int maxSpeed; // 0-100%
};

class DCMotor {
public:
  DCMotor(DcMotorConfig& config );

  void run(int speed);
  void stop();
  void moveTime(int speed, unsigned long time);

private:
  int forwardPin;
  int reversePin;
  int maxSpeed;
  int prevSpeed;
};

#endif