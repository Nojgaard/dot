#pragma once

#include <AsyncUDP.h>

class Comm {
 public:
  bool begin();

 private:
  bool connectToWifi();
  bool connectToController();
  AsyncUDP udp;
};