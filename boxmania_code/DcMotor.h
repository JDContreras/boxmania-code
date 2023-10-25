#ifndef DCMOTOR_H
#define DCMOTOR_H

class DCMotor {
public:
  DCMotor(int forwardPin, int reversePin);

  void run(int speed);
  void stop();

private:
  int forwardPin;
  int reversePin;
};

#endif