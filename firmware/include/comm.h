#pragma once

#include <AsyncUDP.h>

class Comm {
public:
    bool begin();
private:
    AsyncUDP udp;
};