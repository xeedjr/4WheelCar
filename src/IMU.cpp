/*
 * mpu9250->cpp
 *
 *  Created on: Oct 14, 2020
 *      Author: Bogdan
 */

#include <stdio.h>

#include "IMU.h"
#include "mpu9250.h"

/// 10ms tick
#define TICKS_TIMEOUT_S 100

IMU::IMU(MPU9250FIFO *mpu9250, IMUInterface *imu_interface) :
		QActive(Q_STATE_CAST(&IMU::initial)),
		mpu9250(mpu9250),
		imu_interface(imu_interface),
		m_timeEvt(this, kTimer, 0U)
{
	this->start(6U, // priority
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
		//m_timeEvt.armX(TICKS_TIMEOUT_S/50, TICKS_TIMEOUT_S/50);
		status_ = Q_RET_HANDLED;
		break;
	}
	case kDataReady: {
		loop();
		imu_interface->update_data(roll, pitch, heading);
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
    exit(1);
  }

/*  if (mpu9250->calibrateAccel() < 0)
	  exit(1);
  if (mpu9250->calibrateGyro() < 0)
	  exit(1);
  if (mpu9250->calibrateMag() < 0)
	  exit(1);
*/
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
/*
		  printf("%f  %f  %f  %f  %f  %f  %f  %f  %f  %f\n\r",
				  mpu9250->getAccelX_mss(),
				  mpu9250->getAccelY_mss(),
				  mpu9250->getAccelZ_mss(),
				  mpu9250->getGyroX_rads(),
				  mpu9250->getGyroY_rads(),
				  mpu9250->getGyroZ_rads(),
				  mpu9250->getMagX_uT(),
				  mpu9250->getMagY_uT(),
				  mpu9250->getMagZ_uT(),
				  mpu9250->getTemperature_C());
*/
		  printf("Orient: %f\t %f\t %f \n\r", roll, pitch, heading);
    }
}

/*void IMU::loop() {

    // read the sensor
    if(mpu9250->readFifo() == 1) {

    	mpu9250->getFifoAccelX_mss(&aSize, axFifo);
    	mpu9250->getFifoAccelY_mss(&aSize, ayFifo);
    	mpu9250->getFifoAccelZ_mss(&aSize, azFifo);

    	mpu9250->getFifoGyroX_rads(&gSize, gxFifo);
    	mpu9250->getFifoGyroY_rads(&gSize, gyFifo);
    	mpu9250->getFifoGyroZ_rads(&gSize, gzFifo);

    	mpu9250->getFifoMagX_uT(&hSize, hxFifo);
    	mpu9250->getFifoMagY_uT(&hSize, hyFifo);
    	mpu9250->getFifoMagZ_uT(&hSize, hzFifo);

    	for (int i = 0; i < 100; i++)
    	{
    		if (i < aSize) {
    			  printf("%f  %f  %f ",
    					  axFifo[i],
						  ayFifo[i],
						  azFifo[i]);
    		}

    		if (i < gSize) {
    			  printf("%f  %f  %f ",
    					  gxFifo[i],
						  gyFifo[i],
						  gzFifo[i]);
    		}
    		if (i < hSize) {
    			  printf("%f  %f  %f",
    					  hxFifo[i],
						  hyFifo[i],
						  hzFifo[i]);
    			  printf("\n\r");
    		}

    	}
    }
}
*/


