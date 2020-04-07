#pragma once

#include "hal.h"

#define SENSOR_INTERVALS_NUM   (20.0)
#define SENSOR_INTERVAL_RAD   (((3.14*2)/SENSOR_INTERVALS_NUM) * 2)

class VelocityEncoder {
    ioline_t pin;
    float rad_perSec;
    unsigned long timeprev;

    static void VelocityEncoder_encoder(void *arg) {
    	VelocityEncoder *thisp = (VelocityEncoder*)arg;
      auto current = TIME_I2MS(osalOsGetSystemTimeX());
      auto interval_spend_time = current - thisp->timeprev;
      thisp->rad_perSec = ((SENSOR_INTERVAL_RAD * 1.0) / (interval_spend_time / 1000.0));
      thisp->timeprev = current;
    };

  public:
    VelocityEncoder() {};

    void init (ioline_t pin) {
      this->pin = pin;

      palSetLineCallback(pin, VelocityEncoder_encoder, this);
      palEnableLineEvent(pin, PAL_EVENT_MODE_RISING_EDGE);
    };
    ~VelocityEncoder() {};

    float getSpeed() {
    	float ret = 0;

		if (rad_perSec > 0.0) {
			ret = rad_perSec;
			rad_perSec = 0;
		};

        return ret;
    };
};
