/*
 * mpu9250->cpp
 *
 *  Created on: Oct 14, 2020
 *      Author: Bogdan
 */

#include <stdio.h>

#include "Sensors.h"
#include "mpu9250.h"

namespace sensors {

Sensors::Sensors(MPU9250FIFO *mpu9250,
        USSensor *sonars,
        SensorsInterface *interface) :
            mpu9250(mpu9250),
            sonars(sonars),
            interface(interface)
{

}

Sensors::~Sensors() {
}

bool Sensors::initialize(const QP::QEvt *e) {
    // start communication with Sensors
    status = mpu9250->begin();
    if (status < 0) {
      exit(1);
    }


    // etting the accelerometer full scale range to +/-8G
    if (mpu9250->setAccelRange(MPU9250::ACCEL_RANGE_8G) < 0)
        exit(1);
    // setting the gyroscope full scale range to +/-500 deg/s
    if (mpu9250->setGyroRange(MPU9250::GYRO_RANGE_500DPS) < 0)
        exit(1);
    // setting DLPF bandwidth to 20 Hz
    if (mpu9250->setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_41HZ) < 0)
        exit(1);
    // setting SRD to 19 for a 50 Hz update rate
    if (mpu9250->setSrd(19) < 0)
        exit(1);

    // enabling the FIFO to record just the accelerometers
  //  mpu9250->enableFifo(true, true, true, true);

    filter.begin(50);

    if (mpu9250->enableDataReadyInterrupt() < 0)
        exit(1);

    data_irq_is_enabled = true;
}

bool Sensors::imu_loop(const QP::QEvt *e) {
    // read the sensor
    if(mpu9250->readSensor() == 1) {
        // update the filter, which computes orientation
        filter.update(mpu9250->getGyroX_rads()/0.0174533f, mpu9250->getGyroY_rads()/0.0174533f, mpu9250->getGyroZ_rads()/0.0174533f,
                      mpu9250->getAccelX_mss(), mpu9250->getAccelY_mss(), mpu9250->getAccelZ_mss(),
                      mpu9250->getMagX_uT(), mpu9250->getMagY_uT(), mpu9250->getMagZ_uT());

        // print the heading, pitch and roll
        roll = filter.getRoll();
        pitch = filter.getPitch();
        yaw = filter.getYaw();

        interface->update_Sensors_cb(roll, pitch, yaw);
        //          printf("Orient: %f\t %f\t %f \n\r", roll, pitch, yaw);
    }
}

bool Sensors::sonic_process(const QP::QEvt *e) {

    sonars_data[0] = sonars->get_distance(0);
    sonars_data[1] = sonars->get_distance(1);

    interface->us_sensor_cb(sonars_data, 2);
}


}
