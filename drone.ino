#include <SPI.h>

#include <Wire.h>
#include "Gyro.h"
#include "Motor.h"

#include <SPI.h>
#include <LoRa.h>

double map(double x, double in_min, double in_max, double out_min, double out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

enum MESSAGE_TYPE {
  SPEED_VERT
};

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

    double accX, accY, accZ, gyroX, gyroY, gyroZ;
    gyroscope.readRaw(accX, accY, accZ, gyroX, gyroY, gyroZ);
    Serial.print(accX);
    Serial.print(" ");
    Serial.print(accY);
    Serial.print(" ");
    Serial.print(accZ);
    Serial.print(" ");
    Serial.print(gyroX);
    Serial.print(" ");
    Serial.print(gyroY);
    Serial.print(" ");
    Serial.println(gyroZ);
    const VectorFloat normal = gyroscope.normal;
    const float angleX = acos(normal.x) - 3.14 / 2;
    const float angleY = acos(normal.y) - 3.14 / 2;

    const float averagePower = throttle;

    // Serial.print(angleX);
    // Serial.print(" ");
    // Serial.println(angleY);

    const double powerChangeK = 2 / 3.14;

    // Serial.print(max(0, averagePower + (angleY + angleX) / 3.14 * powerChangeK));
    // Serial.print(" ");
    // Serial.print(max(0, averagePower + (angleX - angleY) / 3.14 * powerChangeK));
    // Serial.print(" ");
    // Serial.print(max(0, averagePower - (angleX + angleY) / 3.14 * powerChangeK));
    // Serial.print(" ");
    // Serial.println(max(0, averagePower - (angleX - angleY) / 3.14 * powerChangeK));

    // tMotor.setPower(max(0, averagePower + (angleY + angleX) / 3.14 * powerChangeK));
    // rMotor.setPower(max(0, averagePower + (angleX - angleY) / 3.14 * powerChangeK));
    // bMotor.setPower(max(0, averagePower - (angleX + angleY) / 3.14 * powerChangeK));
    // lMotor.setPower(max(0, averagePower - (angleX - angleY) / 3.14 * powerChangeK));

    // tMotor.setPower(max(0, averagePower));
    // rMotor.setPower(max(0, averagePower));
    // bMotor.setPower(max(0, averagePower));
    // lMotor.setPower(max(0, averagePower));

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

  void handleRCMessage(const String& message) {
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
    // Serial.println("LoRa Receiver");

    // LoRa.setPins(7);

    // if (!LoRa.begin(915E6)) {
    //   Serial.println("Starting LoRa failed!");
    //   while (1)
    //     ;
    // }

    Serial.println("Initializing the gyroscope");

    gyroscope.initialize();

    // Serial.println("Starting the motors");

    // tMotor.start();
    // rMotor.start();
    // bMotor.start();
    // lMotor.start();


    // tMotor.setPower(0);
    // rMotor.setPower(0);
    // bMotor.setPower(0);
    // lMotor.setPower(0);
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
    updateMotors();

    // // try to parse packet
    // int packetSize = LoRa.parsePacket();
    // if (packetSize) {
    //   // received a packet
    //   Serial.print("Received packet ");

    //   String lastRCMessage;

    //   // read packet
    //   while (LoRa.available()) {
    //     lastRCMessage += (char)LoRa.read();
    //   }
    //   Serial.println(lastRCMessage);
    //   handleRCMessage(lastRCMessage);

    //   // print RSSI of packet
    //   Serial.println(LoRa.packetRssi());
    // }
  }
};

Drone drone;

void setup() {
  Serial.begin(19200);
  Serial.println("Hi");
  drone.initialize();
}

void loop() {
  //Serial.println("Update");
  drone.update();
  delay(5);
}
