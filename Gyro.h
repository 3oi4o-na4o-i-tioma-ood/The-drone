class Gyroscope {
private:
  const int MPU = 0x68;
  const double COMPLEMENTARY_FILTER = 0.94;

  double currTime;
  double accXError, accYError, gyroXError, gyroYError, gyroZError;
  double gyroXAngle = 0;
  double gyroYAngle = 0;
  double gyroZAngle = 0;

  void readRaw(double& accX, double& accY, double& accZ, double& gyroX, double& gyroY, double& gyroZ);

  void calcError();

public:
  void initialize();

  double getAngleX();
  double getAngleY();

  void update();
};