#include <Wire.h>
#include <BasicLinearAlgebra.h>

#include "Gyro.h"
#include "Arduino.h"
const int MPU = 0x68;



void Gyroscope::readRaw(double& accX, double& accY, double& accZ, double& gyroX, double& gyroY, double& gyroZ) {
  // Serial.println("Reading data");

  Wire.beginTransmission(MPU);
  Wire.write(0x3B);
  Wire.endTransmission(false);

  Wire.requestFrom(MPU, 6, true);
  accX = (Wire.read() << 8 | Wire.read()) / 16384.0;
  accY = (Wire.read() << 8 | Wire.read()) / 16384.0;
  accZ = (Wire.read() << 8 | Wire.read()) / 16384.0;

  Wire.beginTransmission(MPU);
  Wire.write(0x43);
  Wire.endTransmission(false);

  Wire.requestFrom(MPU, 6, true);
  gyroX = (Wire.read() << 8 | Wire.read()) / 131.0;
  gyroY = (Wire.read() << 8 | Wire.read()) / 131.0;
  gyroZ = (Wire.read() << 8 | Wire.read()) / 131.0;

  // Serial.print(accX);
  // Serial.print(" ");
  // Serial.print(accY);
  // Serial.print(" ");
  // Serial.print(accZ);
  // Serial.print(" ");

  // Serial.print(gyroX);
  // Serial.print(" ");
  // Serial.print(gyroY);
  // Serial.print(" ");
  // Serial.print(gyroZ);
  // Serial.println(" ");
}

void Gyroscope::read(double& accX, double& accY, double& accZ, double& gyroX, double& gyroY, double& gyroZ) {
  readRaw(accX, accY, accZ, gyroX, gyroY, gyroZ);

  gyroX -= gyroXError;
  gyroY -= gyroYError;
  gyroZ -= gyroZError;

  if(abs(gyroX) < 1)
  {
    gyroX = 0;
  }
  
  if(abs(gyroY) < 1)
  {
    gyroY = 0;
  }
  
  if(abs(gyroZ) < 1)
  {
    gyroZ = 0;
  }

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
  Serial.print(gyroZ);
  Serial.println(" ");
}

// void Gyroscope::readDPM(double& rotX, double& rotY, double rotZ)
// {
//   Wire.beginTransmission(MPU);
//   Wire.write(0x35);
//   Wire.endTransmission(false);

//   Wire.requestFrom(MPU, 6, true);
//   gyroX = (Wire.read() << 8 | Wire.read()) / 131.0;
//   gyroY = (Wire.read() << 8 | Wire.read()) / 131.0;
//   gyroZ = (Wire.read() << 8 | Wire.read()) / 131.0;
// }

void Gyroscope::calcError() {
  Serial.println("Calculating the gyroscope error");
  const int trials = 200;

  accXError = 0;
  accYError = 0;
  gyroXError = 0;
  gyroYError = 0;
  gyroZError = 0;

  for (int c = 0; c < trials; ++c) {
    double accX, accY, accZ, gyroX, gyroY, gyroZ;
    readRaw(accX, accY, accZ, gyroX, gyroY, gyroZ);

    accXError += atan(accY / sqrt(pow(accX, 2) + pow(accZ, 2))) * 180 / PI;
    accYError += atan(-accX / sqrt(pow(accY, 2) + pow(accZ, 2))) * 180 / PI;
    gyroXError += gyroX;
    gyroYError += gyroY;
    gyroZError += gyroZ;
  }

  accXError /= trials;
  accYError /= trials;
  gyroXError /= trials;
  gyroYError /= trials;
  gyroZError /= trials;
}

void Gyroscope::initialize() {
  Serial.println("Init gyro");
  Wire.begin();
  Serial.println("Begin");

  Wire.beginTransmission(MPU);
  Serial.println("Begin transmission");

  Wire.write(0x6B);
  Wire.write(0x00);

  Serial.println("Wrote data");
  Wire.endTransmission(true);
  Serial.println("End transmission");


  // Wire.beginTransmission(MPU);
  // Wire.write(0x1C);
  // Wire.write(4);
  // Wire.endTransmission(true);

  // Wire.beginTransmission(MPU);
  // Wire.write(0x1B);
  // Wire.write(0x10);
  // Wire.endTransmission(true);

  calcError();
}

Matrix<3, 3> getRotation(double x, double y, double z) {
  Matrix<3, 3> rotX = {
    1, 0, 0,
    0, cos(x), -sin(x),
    0, sin(x), cos(x)
  };

  Matrix<3, 3> rotY = {
    cos(y), 0, -sin(y),
    0, 1, 0,
    sin(y), 0, cos(y)
  };

  Matrix<3, 3> rotZ = {
    cos(z), -sin(z), 0,
    sin(z), cos(z), 0,
    0, 0, 1
  };

  return rotX * rotY * rotZ;
  // return {
  //   cos(y),
  //   0,
  //   sin(y),
  //   0,
  //   cos(x),
  //   sin(x),
  //   -sin(y),
  //   -sin(x),
  //   cos(x) * cos(y)
  // };
}

void printMatrix(Matrix<3, 3> m)
{
  Serial.print(m(0, 0));
  Serial.print(" ");
  Serial.print(m(0, 1));
  Serial.print(" ");
  Serial.print(m(0, 2));
  Serial.print(" / ");
  Serial.print(m(1, 0));
  Serial.print(" ");
  Serial.print(m(1, 1));
  Serial.print(" ");
  Serial.print(m(1, 2));
  Serial.print(" / ");
  Serial.print(m(2, 0));
  Serial.print(" ");
  Serial.print(m(2, 1));
  Serial.print(" ");
  Serial.print(m(2, 2));
  Serial.print(" ");
}

void Gyroscope::update() {
  double accX, accY, accZ, gyroX, gyroY, gyroZ;
  read(accX, accY, accZ, gyroX, gyroY, gyroZ);

  double prevTime = currTime;
  currTime = millis();
  double dt = (currTime - prevTime) / 1000;

  Matrix<3, 3> rotation = getRotation(gyroX * dt / 100, gyroY * dt / 100, gyroZ * dt / 100);
  orientation *= rotation;

  //printMatrix(rotation);

  //Serial.println();

  // printMatrix(orientation);

    //Serial.println("---------------");

  // velocity += Matrix<3>(accX, accY, accZ) * dt;

  // Serial.print(velocity(0));
  // Serial.print(" ");
  // Serial.print(velocity(1));
  // Serial.print(" ");
  // Serial.print(velocity(2));
  // Serial.print(" ");

  // pos += velocity * dt;
}