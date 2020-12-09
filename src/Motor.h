/*
 * Motor.h
 *
 *  Created on: 30 лист. 2020 р.
 *      Author: Bogdan
 */

#ifndef SRC_MOTOR_H_
#define SRC_MOTOR_H_

#include <functional>
#include "qpcpp.hpp"

#include "TB6612FNG.h"
#include "RPMEncoderOptical.h"

class Motor : public QP::QActive {
	TB6612FNG *drive;
	RPMEncoderOptical *enc1;
	RPMEncoderOptical *enc2;

	enum Signals {
		kSetSpeedL = QP::Q_USER_SIG,
		kSetSpeedR,
		kInitialize,
	    MAX_SIG
	};
public:
	struct Event : public QP::QEvt {
		union {
			uint64_t u64;
			float f;
		} u[5];
		Event(QP::QSignal const s) : QEvt(s) {};
	};
private:
	Event const *current_event  = nullptr;

	uint8_t stack[1024];
    QP::QEvt const *queueSto[10];

	Q_STATE_DECL(initial);
	Q_STATE_DECL(InitializeState);
	Q_STATE_DECL(WaitAPI);

public:
	Motor(TB6612FNG *drive, RPMEncoderOptical *enc1, RPMEncoderOptical *enc2);
	virtual ~Motor();

	void SetSpeedL(float s) {
		auto ev = Q_NEW(Event, kSetSpeedL);
		ev->u[0].f = s;
		POST(ev, this);
	};
	void SetSpeedR(float s) {
		auto ev = Q_NEW(Event, kSetSpeedR);
		ev->u[0].f = s;
		POST(ev, this);
	};
};

#endif /* SRC_MOTOR_H_ */
