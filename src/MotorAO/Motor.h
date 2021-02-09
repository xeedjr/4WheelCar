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

namespace motor {
//26.7 rad/sec on 100pwm
class Motor : public motor::MotorAO {
	TB6612FNG *drive;
	std::function<void(double*, uint8_t)> wheel_position_cb;

	enum WheelType {
	  kL,
	  kR,
	};
	struct Wheel {
	    WheelMotorEncoder *enc;

	    MiniPID *pid;

        float target_wheel_speed = 0;
	    float current_wheel_speed = 0;
	    double prev_curr_pos = 0;
	    float pwm = 0;
	    float pwm_based_on_targed = 0;


	    void init_pid() {
	    }
	    void pid_update() {
	        pwm_based_on_targed = (100.0 / 27.6) * target_wheel_speed;
	        pwm = pid->getOutput(std::abs(current_wheel_speed), std::abs(target_wheel_speed));

	    }
        void enc_update_speed() {
            auto curr_pos = enc->get_wheel_position();
            //printf("%f \n", curr_pos);
            current_wheel_speed = (curr_pos - prev_curr_pos) * 10.0;
            prev_curr_pos = curr_pos;
        }
	} wheel[2];

private:
	Event const *current_event  = nullptr;

	uint8_t stack[1024];
    QP::QEvt const *queueSto[10];

    bool set_speed_left(const QP::QEvt *e);
    bool set_speed_right(const QP::QEvt *e);
    bool pid_init(const QP::QEvt *e);
    bool pid_timeout(const QP::QEvt *e);
    bool get_wheel_speed(const QP::QEvt *e);
public:
	Motor(TB6612FNG *drive, WheelMotorEncoder *enc1, WheelMotorEncoder *enc2, std::function<void(double*, uint8_t)> wheel_position_cb);
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
