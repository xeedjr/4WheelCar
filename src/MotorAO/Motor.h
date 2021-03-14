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

#include "MiniPID.h"
#include "TB6612FNG.h"
#include "WheelMotorEncoder.h"
#include "MotorInterface.h"

namespace motor {
//26.7 rad/sec on 100pwm
class Motor : public motor::MotorAO {
	TB6612FNG *drive;
	MotorInterface *interface;

	enum WheelType {
	  kL,
	  kR,
	};
	struct Wheel {
	    WheelMotorEncoder *enc;

	    MiniPID *pid;

	    bool is_reverse = false;
        float target_wheel_speed = 0;
	    float current_wheel_speed = 0;
	    float current_wheel_pos = 0;
	    double prev_curr_pos = 0;
	    float pwm_total = 0;
	    float pwm = 0;

	    void init_pid() {
	    }
	    void pid_update() {
	        pwm = pid->getOutput(current_wheel_speed, target_wheel_speed);
	    }
        void enc_update_speed() {
            current_wheel_pos = enc->get_wheel_position();
            if (is_reverse)
                current_wheel_pos *= -1.0;
            //printf("%f \n", curr_pos);
            current_wheel_speed = (current_wheel_pos - prev_curr_pos) * 50.0;
            prev_curr_pos = current_wheel_pos;
        }
	} wheel[2];

private:
	Event const *current_event  = nullptr;

	uint8_t stack[10*1024];
    QP::QEvt const *queueSto[10];

    bool set_speed_left(const QP::QEvt *e);
    bool set_speed_right(const QP::QEvt *e);
    bool pid_init(const QP::QEvt *e);
    bool pid_timeout(const QP::QEvt *e);
    bool get_wheel_speed(const QP::QEvt *e);
public:
	Motor(TB6612FNG *drive, WheelMotorEncoder *enc1, WheelMotorEncoder *enc2, MotorInterface *interface);
	virtual ~Motor();

	void startAO() {
	    // TODO Auto-generated constructor stub
	    start(4U, // priority
	                 queueSto, Q_DIM(queueSto),
	#ifndef WIN32
	                 stack, sizeof(stack)); // no stack
	#else
	                 nullptr, 0); // no stack
	#endif
	}

	void SetSpeedL(float rad_sec) {
		auto ev = Q_NEW(Event, SET_SPEED_L_SIG);
		ev->u[0].f = rad_sec;
		POST(ev, this);
	};
	void SetSpeedR(float rad_sec) {
		auto ev = Q_NEW(Event, SET_SPEED_R_SIG);
		ev->u[0].f = rad_sec;
		POST(ev, this);
	};
};
};
#endif /* SRC_MOTOR_H_ */
