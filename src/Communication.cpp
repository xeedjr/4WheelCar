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

Communication::Communication(UART_HandleTypeDef *huart, IMU *imu, Motor *motor) :
		imu(imu),
		huart(huart),
		motor(motor),
		QActive(Q_STATE_CAST(&Communication::initial)),
		m_timeEvt(this, kTimer, 0U)
{
	this_local = this;

	p = (void*)&this->huart;

	in_str_pool.init(in_str_pool_stor, sizeof(in_str_pool_stor), 256);

	if ((str = (char*)in_str_pool.get(0, 0)) == nullptr) {
		exit(0);
	};

	if (HAL_OK != HAL_UART_RegisterCallback(huart, HAL_UART_RX_COMPLETE_CB_ID, [](UART_HandleTypeDef *huart){
		this_local->HAL_UART_RxCpltCallback(huart);
	})) {
		exit(0);
	};

	if (HAL_OK != HAL_UART_Receive_IT(huart, &recv_byte, sizeof(recv_byte))) {
		exit(0);
	};

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

	if (str_len > 200) {
		exit(0);
	}

	if (c == '\n')
	{
		str[str_len] = 0;
		str_len = 0;
		/// linefinished
		auto ev = (Event*)Q_NEW_FROM_ISR(Event,
				kParseResponse);
		ev->u.str = str;
		POST_FROM_ISR(ev, nullptr, this);

		if ((str = (char*)in_str_pool.getFromISR(0, 0)) == nullptr) {
			exit(0);
		};
	}
	else
	{
		str[str_len++] = c;
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
		snprintf(imu_str, sizeof(imu_str), "imu %f %f %f\n", roll, pitch, heading);
		auto res = HAL_UART_Transmit(huart, (uint8_t*)imu_str, strlen(imu_str), 1000);
		if (res != HAL_OK)
			exit(0);
		status_ = Q_RET_HANDLED;
		break;
	}
	case kParseResponse: {
		auto ev = (Event*)e;

		int id = 0;
		sscanf(ev->u.str, "%d ", &id);

		switch(id) {
		case 1: {
			int left = 0, right = 0;
			sscanf(ev->u.str, "%d %d %d", &id, &left, &right);
			motor->SetSpeedL((float)left);
			motor->SetSpeedR((float)right);
			break;
		}
		case 2: {
			break;
		}
		}

		in_str_pool.put(ev->u.str, 0);
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

