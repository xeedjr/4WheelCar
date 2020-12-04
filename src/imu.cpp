/*
 * IMU.cpp
 *
 *  Created on: Oct 14, 2020
 *      Author: Bogdan
 */

#include <stdio.h>

#include "IMU.h"

volatile float roll1, pitch1, heading1;

/// 10ms tick
#define TICKS_TIMEOUT 5

IMU::IMU(MPU9250 *mpu9250) :
		QActive(Q_STATE_CAST(&IMU::initial)),
		mpu9250(mpu9250),
		m_timeEvt(this, kTimer, 0U)
{
	this->start(4U, // priority
				 queueSto, Q_DIM(queueSto),
#ifndef WIN32
				 stack, sizeof(stack)); // no stack
#else
	 	 	 	 nullptr, 0); // no stack
#endif
}

IMU::~IMU() {
}

Q_STATE_DEF(IMU, initial) {
    (void)e; // unused parameter

    return tran(&InitializeState);
}

Q_STATE_DEF(IMU, InitializeState) {
    QP::QState status_;
    switch (e->sig) {
		case Q_ENTRY_SIG: {
			POST(Q_NEW(Event, kInitialize), this);
			status_ = Q_RET_HANDLED;
			break;
		}
		case kInitialize: {
			initialize();
			status_ = tran(&WaitAPI);
			break;
		}
		default: {
			status_ = super(&top);
			break;
		}
	}
	return status_;
}

Q_STATE_DEF(IMU, WaitAPI) {
	QP::QState status_;
	switch (e->sig) {
	case Q_ENTRY_SIG: {
		m_timeEvt.armX(TICKS_TIMEOUT, TICKS_TIMEOUT);
		status_ = Q_RET_HANDLED;
		break;
	}
	case kTimer: {
		loop();
		status_ = Q_RET_HANDLED;
		break;
	}
	default: {
		status_ = super(&top);
		break;
	}
	}
	return status_;
}


void IMU::initialize() {
  // start communication with IMU
  status = mpu9250->begin();
  if (status < 0) {
    throw 1;
  }

  // setting the accelerometer full scale range to +/-8G
  mpu9250->setAccelRange(MPU9250::ACCEL_RANGE_8G);
  // setting the gyroscope full scale range to +/-500 deg/s
  mpu9250->setGyroRange(MPU9250::GYRO_RANGE_500DPS);
  // setting DLPF bandwidth to 20 Hz
  mpu9250->setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_20HZ);
  // setting SRD to 19 for a 50 Hz update rate
  mpu9250->setSrd(19);

  filter.begin(20);
}

void IMU::loop() {

    // read the sensor
    if(mpu9250->readSensor() == 1) {

		// update the filter, which computes orientation
		filter.update(mpu9250->getGyroX_rads()/0.0174533f, mpu9250->getGyroY_rads()/0.0174533f, mpu9250->getGyroZ_rads()/0.0174533f,
					  mpu9250->getAccelX_mss(), mpu9250->getAccelY_mss(), mpu9250->getAccelZ_mss(),
					  mpu9250->getMagX_uT(), mpu9250->getMagY_uT(), mpu9250->getMagZ_uT());

		// print the heading, pitch and roll
		roll = filter.getRoll();
		pitch = filter.getPitch();
		heading = filter.getYaw();

		roll1 = roll;
		pitch1 = pitch;
		heading1 = heading;


		//sprintf(buff, "Orient: %f, %f, %f", roll, pitch, heading);
    }
}



