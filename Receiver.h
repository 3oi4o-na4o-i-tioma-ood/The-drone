#include <RH_ASK.h>

class Receiver {
  RH_ASK driver;
public:
  void init();
  void checkForMessage(char* message);
};