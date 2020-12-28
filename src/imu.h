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

#include "IMUInterface.h"
#include "MPU9250.h"
#include "MadgwickAHRS.h"

class IMU : public QP::QActive {

    IMUInterface *imu_interface;

	enum Signals {
		kTimer = QP::Q_USER_SIG,
		kInitialize,
		kDataReady,
	    MAX_SIG
	};

	struct Event : public QP::QEvt {
		union {
			uint64_t u64;
		} u;
		Event(QP::QSignal const s) : QEvt(s) {};
	};

	Event const *current_event  = nullptr;

	uint8_t stack[2048];
    QP::QEvt const *queueSto[10];
    QP::QTimeEvt m_timeEvt;

	Q_STATE_DECL(initial);
	Q_STATE_DECL(InitializeState);
	Q_STATE_DECL(WaitAPI);



	Madgwick filter;
	MPU9250FIFO *mpu9250;
	int status;
/*
    float axFifo[85], ayFifo[85], azFifo[85];
    size_t aSize;
    float gxFifo[85], gyFifo[85], gzFifo[85];
    size_t gSize;
    float hxFifo[73], hyFifo[73], hzFifo[73];
    size_t hSize;
*/
	float roll, pitch, heading;
	char buff[50];

	void initialize();
	void loop();
public:
	IMU(MPU9250FIFO *mpu9250, IMUInterface *imu_interface);
	virtual ~IMU();

	void data_ready() {
		POST_FROM_ISR(Q_NEW_FROM_ISR(Event, kDataReady), nullptr, this);
	};
};

#endif /* SRC_IMU_H_ */
