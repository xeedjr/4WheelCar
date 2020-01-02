#pragma once

#include "board.h"

class VelocityEncoder {
    int _pin;
    static float rad_perSecL;
    static float rad_perSecR;
    static unsigned long timeLprev;
    static unsigned long timeRprev;

    static void encoderL() {
      auto current = millis();
      auto spend_time = current - timeLprev;
      rad_perSecL = (((2.0*3.14)/12.0) * 1.0) / (spend_time/1000.0);
      timeLprev = millis();
    };
    static void encoderR() {
      Serial.println("interr");
      auto current = millis();
      auto spend_time = current - timeRprev;
      rad_perSecR = (((2.0*3.14)/12.0) * 1.0) / (spend_time/1000.0);
      timeRprev = millis();
    };
  public:
    VelocityEncoder(int pin) {
      _pin = pin;
      pinMode(_pin, INPUT_PULLUP);
      if (_pin == ENCR){
        attachInterrupt(digitalPinToInterrupt(pin), VelocityEncoder::encoderR, CHANGE);
      }
      if (_pin == ENCL) {
        attachInterrupt(digitalPinToInterrupt(pin), VelocityEncoder::encoderL, CHANGE);
      }
    };
    ~VelocityEncoder() {};

    float getSpeed() {
      float ret = 0;
      if (_pin == ENCL) {
        if (rad_perSecL > 0.0) {
            ret = rad_perSecL;
            rad_perSecL = 0;
        };
      }
      if (_pin == ENCR) {
        if (rad_perSecR > 0.0) {
            ret = rad_perSecR;
            rad_perSecR = 0;
        };
      }
      
      return ret;
    };
};

float VelocityEncoder::rad_perSecL = 0;
float VelocityEncoder::rad_perSecR = 0;
unsigned long VelocityEncoder::timeLprev = 0;
unsigned long VelocityEncoder::timeRprev = 0;
