/*
 * Communication.h
 *
 *  Created on: Dec 6, 2020
 *      Author: Bogdan
 */

#ifndef COMMUNICATION_H_
#define COMMUNICATION_H_

#include <functional>
#include "qpcpp.hpp"
#include "usart.h"
#include "IMU.h"

class Communication : public QP::QActive {

	IMU *imu;
	UART_HandleTypeDef *huart;
	void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);

	float roll, pitch, heading;
	char imu_str[50];

	uint8_t recv_byte;

//*** QP Stuff ***//
	struct Event : public QP::QEvt {
		union {
			uint64_t u64;
		} u;
		Event(QP::QSignal const s) : QEvt(s) {};
	};

	enum Signals {
		kTimer = QP::Q_USER_SIG,
		kInitialize,
	    MAX_SIG
	};

	Event const *current_event  = nullptr;
	uint8_t stack[1024];
    QP::QEvt const *queueSto[10];
    QP::QTimeEvt m_timeEvt;

	Q_STATE_DECL(initial);
	Q_STATE_DECL(InitializeState);
	Q_STATE_DECL(WaitAPI);
//*** QP Stuff ***//


public:
	Communication(UART_HandleTypeDef *huart, IMU *imu);
	virtual ~Communication();
};

#endif /* COMMUNICATION_H_ */
