#include <Servo.h>
#include "Motor.h"
#include "Arduino.h"

Motor::Motor(int pin)
  : pin(pin) {
  servo.attach(pin);
}

Motor::~Motor() {
  servo.detach();
}

// power is from 0 to 1
void Motor::setPower(double power) {
  if (power < 0 || power > 1) {
    Serial.print("Error: ivalid value passed to Motor::setPower. Expected a value between 0 and 1, but got ");
    Serial.println(power);
    return;
  }

  servo.write(power * 180);
}

double Motor::getPower() const {
  return (double)servo.read() / 180;
}

void Motor::start() {
  servo.write(20);

  // To do: get rid of the thread-blocking delay
  delay(500);
  
  servo.write(0);
}