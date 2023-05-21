class Gyroscope {
private:
  const int MPU = 0x68;
  const double COMPLEMENTARY_FILTER = 0.94;

  double currTime;
  double accXError, accYError, gyroXError, gyroYError, gyroZError;
  double gyroXAngle = 0;
  double gyroYAngle = 0;
  double gyroZAngle = 0;

  void readRaw(double& accX, double& accY, double& accZ, double& gyroX, double& gyroY, double& gyroZ) {
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
  }

  void calcError() {
    auto trials = 200;

    accXError = 0;
    accYError = 0;
    gyroXError = 0;
    gyroYError = 0;
    gyroZError = 0;

    for (auto c = 0; c < trials; ++c) {
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

public:
  void initialize() {
    Wire.begin();

    Wire.beginTransmission(MPU);
    Wire.write(0x6B);
    Wire.write(0x00);
    Wire.endTransmission(true);

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

  void read(double& roll, double& pitch, double& yaw) {
      double accX, accY, accZ, gyroX, gyroY, gyroZ;
      readRaw(accX, accY, accZ, gyroX, gyroY, gyroZ);

      double prevTime = currTime;
      currTime = millis();
      double elapsedTime = (currTime - prevTime) / 1000;

      double accXAngle = atan(accY / sqrt(pow(accX, 2) + pow(accZ, 2))) * 180 / PI - accXError;
      double accYAngle = atan(-accX / sqrt(pow(accY, 2) + pow(accZ, 2))) * 180 / PI - accYError;
      gyroXAngle += (gyroX - gyroXError) * elapsedTime;
      gyroYAngle += (gyroY - gyroYError) * elapsedTime;
      gyroZAngle += (gyroZ - gyroZError) * elapsedTime;

      roll = COMPLEMENTARY_FILTER * gyroXAngle + (1 - COMPLEMENTARY_FILTER) * accXAngle;
      pitch = COMPLEMENTARY_FILTER * gyroYAngle + (1 - COMPLEMENTARY_FILTER) * accYAngle;
      yaw = gyroZAngle;
  }
};