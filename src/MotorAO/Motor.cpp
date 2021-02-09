/*
 * Motor.cpp
 *
 *  Created on: 30 лист. 2020 р.
 *      Author: Bogdan
 */
#include <math.h>
#include <Motor.h>

namespace motor {

Motor::Motor(TB6612FNG *drive, WheelMotorEncoder *enc1, WheelMotorEncoder *enc2, std::function<void(double*, uint8_t)> wheel_position_cb) :
	drive(drive),
	wheel_position_cb(wheel_position_cb)
{
    wheel[kL].enc = enc1;
    wheel[kR].enc = enc2;

    wheel[kR].pid = new MiniPID(5, 2, 2);
    wheel[kR].pid->setOutputLimits(-50, 50);

    wheel[kL].pid = new MiniPID(5, 4, 0.1);
    wheel[kL].pid->setOutputLimits(-50, 50);

 //   wheel[kL].target_wheel_speed = 5.0;
}

Motor::~Motor() {
	// TODO Auto-generated destructor stub
}

bool Motor::set_speed_left(const QP::QEvt *e) {
    auto ev = (Event*)e;

    wheel[kL].target_wheel_speed = ev->u[0].f;
/*
    float speed = ev->u[0].f;
    float pwm = (100.0/1.0)*fabs(speed);
    TB6612FNG::Mode mode = TB6612FNG::Mode::kCW;
    if (speed > 0) {
        mode = TB6612FNG::Mode::kCCW;
    }
    drive->drive(TB6612FNG::Channels::kA, mode, pwm);
    */
    return true;
};

bool Motor::set_speed_right(const QP::QEvt *e) {
    auto ev = (Event*)e;

    wheel[kR].target_wheel_speed = ev->u[0].f;

    return true;
};

bool Motor::pid_init(const QP::QEvt *e) {
    auto ev = (Event*)e;

    wheel[kL].init_pid();
    wheel[kR].init_pid();

    return true;
};

bool Motor::pid_timeout(const QP::QEvt *e) {
    auto ev = (Event*)e;

    wheel[kL].pid_update();
    wheel[kR].pid_update();

    drive->drive(TB6612FNG::Channels::kB, TB6612FNG::Mode::kCCW, wheel[kL].pwm_based_on_targed + wheel[kL].pwm);
    drive->drive(TB6612FNG::Channels::kA, TB6612FNG::Mode::kCCW, wheel[kL].pwm_based_on_targed + wheel[kR].pwm);

    //printf("%f %f %f %f\n", std::abs(wheel[kL].target_wheel_speed), std::abs(wheel[kL].current_wheel_speed),  wheel[kL].pwm_based_on_targed, wheel[kL].pwm);

    return true;
};

bool Motor::get_wheel_speed(const QP::QEvt *e) {
    auto ev = (Event*)e;

    /// perios 100 ms

    wheel[kL].enc_update_speed();
    wheel[kR].enc_update_speed();

    return true;
};


}
