/*
 * Motor.cpp
 *
 *  Created on: 30 лист. 2020 р.
 *      Author: Bogdan
 */

#include "Motor.h"

Motor::Motor(TB6612FNG *drive, RPMEncoderOptical *enc1, RPMEncoderOptical *enc2) :
	QActive(Q_STATE_CAST(&Motor::initial)),
	drive(drive), enc1(enc1), enc2(enc2)
{
	// TODO Auto-generated constructor stub
	this->start(4U, // priority
				 queueSto, Q_DIM(queueSto),
#ifndef WIN32
				 stack, sizeof(stack)); // no stack
#else
	 	 	 	 nullptr, 0); // no stack
#endif
}

Motor::~Motor() {
	// TODO Auto-generated destructor stub
}

Q_STATE_DEF(Motor, initial) {
    (void)e; // unused parameter

    return tran(&InitializeState);
}

Q_STATE_DEF(Motor, InitializeState) {
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

Q_STATE_DEF(Motor, WaitAPI) {
	QP::QState status_;
	switch (e->sig) {
	case kSetSpeed: {
		drive->drive(TB6612FNG::Channels::kA, TB6612FNG::Mode::kCCW, 20);
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

