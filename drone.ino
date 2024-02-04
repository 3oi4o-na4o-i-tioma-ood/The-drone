#include <SPI.h>

#include <Wire.h>
#include "Gyro.h"
#include "Motor.h"
#include "./RH_ASK.h"

double map(double x, double in_min, double in_max, double out_min, double out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

enum MESSAGE_TYPE {
  SPEED_VERT
};

class Drone {
  RH_ASK RC_driver = RH_ASK(500, 11);

  uint8_t lastRCMessage[RH_ASK_MAX_MESSAGE_LEN];

  float throttle = 0;

  Gyroscope gyroscope;
  Receiver receiver;

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

    const float averagePower = 0.5;

    Serial.print(angleX);
    Serial.print(" ");
    Serial.println(angleY);

    const double powerChangeK = averagePower / 3.14 * 2;

    Serial.println(averagePower + (angleY + angleX) * powerChangeK);
    Serial.println(averagePower + (angleX - angleY) * powerChangeK);
    Serial.println(averagePower - (angleX + angleY) * powerChangeK);
    Serial.println(averagePower - (angleX - angleY) * powerChangeK);

    // tMotor.setPower(averagePower + (angleY + angleX) / 3.14 * powerChangeK);
    // rMotor.setPower(averagePower + (angleX - angleY) / 3.14 * powerChangeK);
    // bMotor.setPower(averagePower - (angleX + angleY) / 3.14 * powerChangeK);
    // lMotor.setPower(averagePower - (angleX - angleY) / 3.14 * powerChangeK);

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

  void handleRCMessage() {
    String message = String((char*)lastRCMessage);
    Serial.print("Got message: ");
    Serial.println(message);

    // 1 command + max 9 parameters because why not
    String items[10];
    int itemsCount = 0;
    int lastIndex = 0;
    int index = 0;
    while (index != -1) {
      index = message.indexOf(' ', lastIndex);
      items[itemsCount++] = index == -1 ? message.substring(lastIndex) : message.substring(lastIndex, index);
      lastIndex = index + 1;  // Skip the space
    }

    const int command = items[0].toInt();

    switch (command) {
      case SPEED_VERT:
        {
          Serial.println(items[1]);
          const float newValue = items[1].toFloat() / 100.0;
          Serial.print("Setting throttle to: ");
          Serial.println(newValue);
          setThrottle(newValue);
          break;
        }
    }
  }

public:
  void initialize() {
    Serial.println("Initializing the gyroscope");

    //gyroscope.initialize();

    Serial.println("Starting the motors");

    delay(1000);
    tMotor.start();
    rMotor.start();
    bMotor.start();
    lMotor.start();


    RC_driver.init();
    RC_driver.setModeRx();


    tMotor.setPower(0);
    rMotor.setPower(0);
    bMotor.setPower(0);
    lMotor.setPower(0);
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
    //gyroscope.update();
    updateMotors();

    uint8_t buflen = sizeof(lastRCMessage);
    if (RC_driver.recv(lastRCMessage, &buflen))  // Non-blocking
    {
      Serial.print("1Got message: ");
      Serial.println((char*)lastRCMessage);
      handleRCMessage();
    }
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
  delay(50);
}
