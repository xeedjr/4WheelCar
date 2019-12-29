#pragma once

#include "board.h"

class VelocityEncoder {
    int _pin;
    static int count1;
    static int count2;

    float speed1;
    float speed2;

    static void encoder1() {
      count1++;
    };
    static void encoder2() {
      count2++;
    };
  public:
    VelocityEncoder(int pin) {
      _pin = pin;
      pinMode(_pin, INPUT_PULLUP);
      if (_pin == ENCR){
        count1 = 0;
        attachInterrupt(digitalPinToInterrupt(pin), VelocityEncoder::encoder1, CHANGE);
      }
      if (_pin == ENCL) {
        count2 = 0;
        attachInterrupt(digitalPinToInterrupt(pin), VelocityEncoder::encoder2, CHANGE);
      }
    };
    ~VelocityEncoder() {};

    int every_second () {
      if (_pin == ENCR){
        speed1 = count1 * 2*3.14 / 12;
        count1 = 0;
      }
      if (_pin == ENCL) {
        speed2 = count2 * 2*3.14 / 12;
        count2 = 0;
      }
    };
    
    float getSpeed() {
      if (_pin == ENCR){
        return speed1;
      }
      if (_pin == ENCL) {
        return speed2;
      }
    };
};

int VelocityEncoder::count1 = 0;
int VelocityEncoder::count2 = 0;
