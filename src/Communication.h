/*
 * Communication.h
 *
 *  Created on: Dec 6, 2020
 *      Author: Bogdan
 */

#ifndef COMMUNICATION_H_
#define COMMUNICATION_H_

#include <Motor.h>
#include <functional>
#include "qpcpp.hpp"
#include "usart.h"
#include "IMUInterface.h"

class Communication : public QP::QActive, public IMUInterface {
	void *p;
	motor::Motor *motor;
	UART_HandleTypeDef *huart;
	void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);

	float roll, pitch, heading;
	char imu_str[50];

	uint8_t recv_byte;
	char *str;
	uint8_t str_len = 0;

//*** QP Stuff ***//
	struct Event : public QP::QEvt {
		union {
			uint64_t u64;
			char *str;
			float f;
		} u[5];
		Event(QP::QSignal const s) : QEvt(s) {};
	};

	enum Signals {
		kTimer = QP::Q_USER_SIG,
		kInitialize,
		kParseResponse,
		kIMUUpdateData,
	    MAX_SIG
	};

	Event const *current_event  = nullptr;
	uint8_t stack[1024];
    QP::QEvt const *queueSto[10];
    QP::QTimeEvt m_timeEvt;
	QP::QMPool in_str_pool;
	uint8_t in_str_pool_stor[1024];

	Q_STATE_DECL(initial);
	Q_STATE_DECL(InitializeState);
	Q_STATE_DECL(WaitAPI);
//*** QP Stuff ***//


public:
	Communication(UART_HandleTypeDef *huart, motor::Motor *motor);
	virtual ~Communication();

	void startAO() {
	    start(8U, // priority
	                 queueSto, Q_DIM(queueSto),
	#ifndef WIN32
	                 stack, sizeof(stack)); // no stack
	#else
	                 nullptr, 0); // no stack
	#endif
	}

	void update_data (float pitch, float roll, float heading) {
        auto ev = Q_NEW(Event, kIMUUpdateData);
        ev->u[0].f = pitch;
        ev->u[1].f = roll;
        ev->u[2].f = heading;
        POST(ev, this);
	}
};

#endif /* COMMUNICATION_H_ */
