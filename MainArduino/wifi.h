#pragma once

#include <stdint.h>

// git clone https://github.com/itead/ITEADLIB_Arduino_WeeESP8266.git
class WIFI {
  uint8_t _buffer[256];
  uint32_t _len = 0;
  uint32_t _index = 0;
public:  
    WIFI() {};
    ~WIFI() {};

   int read ();
   void send (uint8_t* data, int length);
    void setup();
    void loop ();
};
