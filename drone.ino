#include <Wire.h>
#include "Gyro.h"
#include "Motor.h"

double map(double x, double in_min, double in_max, double out_min, double out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

class Drone {
  float throttle = 0;
  Gyroscope gyroscope;
  Motor tMotor = Motor(3);   // blue
  Motor rMotor = Motor(6);   // red
  Motor bMotor = Motor(9);   // green
  Motor lMotor = Motor(10);  // yellow

  void updateMotors() {
    // double roll, pitch, yaw;

    // roll -> long side, tilting to INT increases
    // pitch -> short side, tilting to ITG/MPU increases
    // yaw -> rotating counterclockwise increases

    // No more than half rotation
    // roll = clamp(roll, -200, 200, -0.5, 0.5);
    // pitch = clamp(pitch, -200, 200, -0.5, 0.5);
    const VectorFloat normal = gyroscope.normal;
    const float angleX = acos(normal.x) - 3.14 / 2;
    const float angleY = acos(normal.y) - 3.14 / 2;

    const float averagePower = 0.35;

    Serial.print(angleX);
    Serial.print(" ");
    Serial.println(angleY);

    Serial.println(averagePower + (angleY + angleX) / 3.14);
    Serial.println(averagePower + (angleX - angleY) / 3.14);
    Serial.println(averagePower - (angleX + angleY) / 3.14);
    Serial.println(averagePower - (angleX - angleY) / 3.14);

    tMotor.setPower(averagePower + (angleY + angleX) / 3.14);
    rMotor.setPower(averagePower + (angleX - angleY) / 3.14);
    bMotor.setPower(averagePower - (angleX + angleY) / 3.14);
    lMotor.setPower(averagePower - (angleX - angleY) / 3.14);

    Matrix<3> pos = gyroscope.pos;

    // Serial.print(pos(0));
    // Serial.print(" ");
    // Serial.print(pos(1));
    // Serial.print(" ");
    // Serial.println(pos(2));

    // Serial.print("Top: ");
    // Serial.print(tMotor.getPower());
    // Serial.print(", Right: ");
    // Serial.print(rMotor.getPower());
    // Serial.print(", Bottom: ");
    // Serial.print(bMotor.getPower());
    // Serial.print(", Left: ");
    // Serial.print(lMotor.getPower());
    // Serial.println();
  }

public:
  void initialize() {
    Serial.println("Initializing the gyroscope");

    gyroscope.initialize();

    Serial.println("Starting the motors");

delay(1000);
    tMotor.start();
    rMotor.start();
    bMotor.start();
    lMotor.start();
    // delay(5000);
  }

  void setThrottle(float throttle) {
    if (throttle < 0 || throttle > 1) {
      Serial.print("Error: ivalid value passed to Drone::setThrottle. Expected a value between 0 and 1, but got ");
      Serial.println(throttle);
      return;
    }

    this->throttle = throttle;
  }

  // void goUp() {
  //   rMotor.setPower(0.75);
  //   tMotor.setPower(0.75);
  //   bMotor.setPower(0.75);
  //   lMotor.setPower(0.75);
  // }


  void update() {
    gyroscope.update();
  //tMotor.setPower(0.4);
    updateMotors();
  }
};

Drone drone;

void setup() {
  Serial.begin(19200);
  Serial.println("Hi");
  drone.initialize();
}

void loop() {
  Serial.println("Update");
  drone.update();
  delay(300);
}
