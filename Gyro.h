#include <BasicLinearAlgebra.h>
#include <ElementStorage.h>

using namespace BLA;

class Gyroscope {
private:
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
  // void readDPM(double& rotX, double& rotY, double rotZ);

  void calcError();

public:
  void initialize();

  double getAngleX();
  double getAngleY();

  void update();
};