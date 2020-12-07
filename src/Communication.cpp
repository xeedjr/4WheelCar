/*
 * Communication.cpp
 *
 *  Created on: Dec 6, 2020
 *      Author: Bogdan
 */
#include <string.h>

#include <Communication.h>

/// 10ms tick
#define TICKS_TIMEOUT_S (100)

static Communication *this_local = nullptr;

Communication::Communication(UART_HandleTypeDef *huart, IMU *imu) :
		imu(imu),
		huart(huart),
		QActive(Q_STATE_CAST(&Communication::initial)),
		m_timeEvt(this, kTimer, 0U)
{
	this_local = this;

/*	if (HAL_OK != HAL_UART_RegisterCallback(huart, HAL_UART_RX_COMPLETE_CB_ID, [](UART_HandleTypeDef *huart){
		this_local->HAL_UART_RxCpltCallback(huart);
	})) {
		exit(0);
	};

	if (HAL_OK != HAL_UART_Receive_IT(huart, &recv_byte, sizeof(recv_byte))) {
		exit(0);
	};
*/
	this->start(8U, // priority
				 queueSto, Q_DIM(queueSto),
#ifndef WIN32
				 stack, sizeof(stack)); // no stack
#else
	 	 	 	 nullptr, 0); // no stack
#endif
}

Communication::~Communication() {
	// TODO Auto-generated destructor stub
}

void Communication::HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	char c = recv_byte;

	auto ret = HAL_UART_Receive_IT(huart, &recv_byte, sizeof(recv_byte));
	if (HAL_OK != ret) {
		exit(0);
	};
}

Q_STATE_DEF(Communication, initial) {
    (void)e; // unused parameter

    return tran(&InitializeState);
}

Q_STATE_DEF(Communication, InitializeState) {
    QP::QState status_;
    switch (e->sig) {
		case Q_ENTRY_SIG: {
			POST(Q_NEW(Event, kInitialize), this);
			status_ = Q_RET_HANDLED;
			break;
		}
		case kInitialize: {



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

Q_STATE_DEF(Communication, WaitAPI) {
	QP::QState status_;
	switch (e->sig) {
	case Q_ENTRY_SIG: {
		m_timeEvt.armX(TICKS_TIMEOUT_S/20, TICKS_TIMEOUT_S/20);
		status_ = Q_RET_HANDLED;
		break;
	}
	case kTimer: {
		imu->get_current(roll, pitch, heading);
		sprintf(imu_str, "Orient: %f, %f, %f\n\r", roll, pitch, heading);
		HAL_UART_Transmit(huart, (uint8_t*)imu_str, strlen(imu_str), 1000);
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

