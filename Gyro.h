#include <BasicLinearAlgebra.h>
#include <ElementStorage.h>
  #include "MPU6050_6Axis_MotionApps20.h"

using namespace BLA;

class Gyroscope {
private:
  MPU6050 mpu;

  const int MPU = 0x68;
  const double COMPLEMENTARY_FILTER = 0.94;
public:
  double currTime;
  double accXError, accYError, gyroXError, gyroYError, gyroZError;
  BLA::Matrix<3, 3> orientation = {
    1, 0, 0,
    0, 1, 0,
    0, 0, 1
  };
  BLA::Matrix<3> velocity;
  BLA::Matrix<3> pos;

  void readRaw(double& accX, double& accY, double& accZ, double& gyroX, double& gyroY, double& gyroZ);
  void read(double& accX, double& accY, double& accZ, double& gyroX, double& gyroY, double& gyroZ);
  void readDMP(double& x, double& i, double& j, double& k);

  void calcError();

public:
  void initialize();

  double getAngleX();
  double getAngleY();

  void update();
};