#pragma once

#include "hal.h"

#define SENSOR_INTERVALS_NUM   (20.0)
#define SENSOR_INTERVAL_RAD   (((3.14*2)/SENSOR_INTERVALS_NUM) * 2)

class VelocityEncoder {
    ioline_t pin;
    static float rad_perSecL;
    static float rad_perSecR;
    static unsigned long timeLprev;
    static unsigned long timeRprev;

    static void encoderL() {
      auto current = millis();
      auto interval_spend_time = current - timeLprev;
      rad_perSecL = ((SENSOR_INTERVAL_RAD * 1.0) / (interval_spend_time / 1000.0));
      timeLprev = current;
    };
    static void encoderR() {
      //Serial.println("interr");
      auto current = millis();
      auto interval_spend_time = current - timeRprev;
      rad_perSecR = ((SENSOR_INTERVAL_RAD * 1.0) / (interval_spend_time / 1000.0));
      timeRprev = current;
    };
  public:
    VelocityEncoder(ioline_t pin) {
      this->pin = pin;
      palSetLineMode(pin, PAL_MODE_INPUT_PULLUP);

      if (pin == ENCL) {
        attachInterrupt(digitalPinToInterrupt(pin), VelocityEncoder::encoderL, RISING );
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
