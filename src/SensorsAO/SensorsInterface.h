/*
 * SensorsInterface.h
 *
 *  Created on: 28 груд. 2020 р.
 *      Author: Bogdan
 */

#pragma once

namespace sensors {

class SensorsInterface {
public:
    virtual void update_Sensors_cb(float, float, float) = 0;
    virtual void us_sensor_cb(float*, uint8_t) = 0;
    virtual void tof_sensors_cb(float*, uint8_t) = 0;
};
};
