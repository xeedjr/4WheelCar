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
    virtual void update_Sensors_cb(float imu_angle_alpha,
            float imu_angle_beta,
            float imu_angle_gamma,
            float imu_vel_alpha,
            float imu_vel_beta,
            float imu_vel_gamma,
            float imu_acc_x,
            float imu_acc_y,
            float imu_acc_z) = 0;
    virtual void us_sensor_cb(float*, uint8_t) = 0;
    virtual void tof_sensors_cb(float*, uint8_t) = 0;
};
};
