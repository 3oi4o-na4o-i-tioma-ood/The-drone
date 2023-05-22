#include <Wire.h>

#include "Gyro.h"
#include "Arduino.h"
const int MPU = 0x68;

void Gyroscope::readRaw(double& accX, double& accY, double& accZ, double& gyroX, double& gyroY, double& gyroZ) {
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

    Serial.println("Read data");

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

void Gyroscope::update()
{
  double accX, accY, accZ, gyroX, gyroY, gyroZ;
  readRaw(accX, accY, accZ, gyroX, gyroY, gyroZ);

  double prevTime = currTime;
  currTime = millis();
  double elapsedTime = (currTime - prevTime) / 1000;

  gyroXAngle += (gyroX - gyroXError) * elapsedTime;
  gyroYAngle += (gyroY - gyroYError) * elapsedTime;
  gyroZAngle += (gyroZ - gyroZError) * elapsedTime;
}

double Gyroscope::getAngleX()
{
  return gyroXAngle;
}


double Gyroscope::getAngleY()
{
  return gyroYAngle;
}