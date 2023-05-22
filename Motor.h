#include <Servo.h>

class Motor {
  Servo servo;

public:
  const int pin;

  Motor(int pin);
  ~Motor();

  // power is from 0 to 1
  void setPower(double power);
  double getPower() const;

  void start();
};