/*
 * IMU.h
 *
 *  Created on: Oct 14, 2020
 *      Author: Bogdan
 */

#ifndef SRC_IMU_H_
#define SRC_IMU_H_

//#include <cmsis_os.h>
#include <functional>
#include "qpcpp.hpp"

#include "MPU9250.h"
#include "MadgwickAHRS.h"

class IMU : public QP::QActive {

	enum Signals {
		kTimer = QP::Q_USER_SIG,
		kInitialize,
	    MAX_SIG
	};

	struct Event : public QP::QEvt {
		union {
			uint64_t u64;
		} u;
		Event(QP::QSignal const s) : QEvt(s) {};
	};

	Event const *current_event  = nullptr;

	uint8_t stack[1024];
    QP::QEvt const *queueSto[10];
    QP::QTimeEvt m_timeEvt;

	Q_STATE_DECL(initial);
	Q_STATE_DECL(InitializeState);
	Q_STATE_DECL(WaitAPI);



	Madgwick filter;
	MPU9250 *mpu9250;
	int status;

	float roll, pitch, heading;
	char buff[50];

	void initialize();
	void loop();
public:
	IMU(MPU9250 *mpu9250);
	virtual ~IMU();
};

#endif /* SRC_IMU_H_ */
