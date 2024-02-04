#include "Receiver.h"

void Receiver::init()
{
  driver.init();
}

void Receiver::checkForMessage(char* message)
{
  uint8_t buf[128];
  uint8_t buflen = sizeof(buf);

  if (driver.recv(buf, &buflen)) {

    Serial.print("Message Received: ");
    Serial.println((char *)buf);
  }
}