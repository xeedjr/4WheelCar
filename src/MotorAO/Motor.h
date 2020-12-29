/*
 * Motor.h
 *
 *  Created on: 30 лист. 2020 р.
 *      Author: Bogdan
 */

#ifndef SRC_MOTOR_H_
#define SRC_MOTOR_H_

#include <functional>
#include <math.h>       /* fabs */

#include "MotorAO.h"

#include "pid.h"
#include "TB6612FNG.h"
#include "RPMEncoderOptical.h"

namespace motor {

class Motor : public motor::MotorAO {
	TB6612FNG *drive;
	RPMEncoderOptical *enc1;
	RPMEncoderOptical *enc2;

	/// PID
	unsigned long pidPeriod = 500;
	unsigned long setpoint = 512;
	int32_t kp = 1011120;
	int32_t ki = 1320*1000;
	int32_t kd = 5280 * 1000;
	uint8_t qn = 22;    // Set QN to 32 - DAC resolution
	Pid::PID pidController = Pid::PID(setpoint, kp, ki, kd, qn);

private:
	Event const *current_event  = nullptr;

	uint8_t stack[1024];
    QP::QEvt const *queueSto[10];

    bool set_speed_left(const QP::QEvt *e);
    bool set_speed_right(const QP::QEvt *e);
    bool pid_init(const QP::QEvt *e);
    bool pid_timeout(const QP::QEvt *e);
public:
	Motor(TB6612FNG *drive, RPMEncoderOptical *enc1, RPMEncoderOptical *enc2);
	virtual ~Motor();

	void SetSpeedL(float s) {
		auto ev = Q_NEW(Event, SET_SPEED_L_SIG);
		ev->u[0].f = s;
		POST(ev, this);
	};
	void SetSpeedR(float s) {
		auto ev = Q_NEW(Event, SET_SPEED_R_SIG);
		ev->u[0].f = s;
		POST(ev, this);
	};
};
};
#endif /* SRC_MOTOR_H_ */
