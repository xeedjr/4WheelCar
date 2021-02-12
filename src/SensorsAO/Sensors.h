/*
 * Sensors.h
 *
 *  Created on: Oct 14, 2020
 *      Author: Bogdan
 */

#pragma once

//#include <cmsis_os.h>
#include <functional>
#include "qpcpp.hpp"

#include "MPU9250.h"
#include "USSensor.h"
#include "MadgwickAHRS.h"
#include "SensorsAO.h"
#include "SensorsInterface.h"

namespace sensors {

class Sensors : public sensors::SensorsAO {

    SensorsInterface *interface;

	Event const *active_event  = nullptr;

	uint8_t stack[2048];
    QP::QEvt const *queueSto[25];

	Madgwick filter;
	MPU9250FIFO *mpu9250;
	int status;

	USSensor *sonars;
	float sonars_data[3];

	float roll, pitch, yaw;
	char buff[50];
	bool data_irq_is_enabled = false;

    bool initialize(const QP::QEvt *e);
    bool imu_loop(const QP::QEvt *e);
    bool sonic_process(const QP::QEvt *e);

public:
	Sensors(MPU9250FIFO *mpu9250,
	        USSensor *sonars,
	        SensorsInterface *interface);
	virtual ~Sensors();

	void startAO() {
	    start(6U, // priority
	                 queueSto, Q_DIM(queueSto),
	#ifndef WIN32
	                 stack, sizeof(stack)); // no stack
	#else
	                 nullptr, 0); // no stack
	#endif
	}

	void data_ready() {
	    if (!data_irq_is_enabled)
	        return;

		POST_FROM_ISR(Q_NEW_FROM_ISR(Event, IMU_DATA_READY_SIG), nullptr, this);
	};
};
};
