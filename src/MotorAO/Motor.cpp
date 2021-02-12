/*
 * Motor.cpp
 *
 *  Created on: 30 лист. 2020 р.
 *      Author: Bogdan
 */
#include <math.h>
#include <Motor.h>

namespace motor {

Motor::Motor(TB6612FNG *drive, WheelMotorEncoder *enc1, WheelMotorEncoder *enc2, MotorInterface *interface) :
	drive(drive),
	interface(interface)
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
    auto L_direction = TB6612FNG::Mode::kCCW;
    auto R_direction = TB6612FNG::Mode::kCCW;

    wheel[kL].pid_update();
    wheel[kR].pid_update();

    if (wheel[kL].target_wheel_speed > 0)
        L_direction = TB6612FNG::Mode::kCW;
    if (wheel[kR].target_wheel_speed > 0)
        R_direction = TB6612FNG::Mode::kCW;

    drive->drive(TB6612FNG::Channels::kB, L_direction, wheel[kL].pwm_based_on_targed + wheel[kL].pwm);
    drive->drive(TB6612FNG::Channels::kA, R_direction, wheel[kR].pwm_based_on_targed + wheel[kR].pwm);

    //printf("%f %f %f %f\n", std::abs(wheel[kL].target_wheel_speed), std::abs(wheel[kL].current_wheel_speed),  wheel[kL].pwm_based_on_targed, wheel[kL].pwm);

    return true;
};

bool Motor::get_wheel_speed(const QP::QEvt *e) {
    auto ev = (Event*)e;

    /// perios 100 ms

    wheel[kL].enc_update_speed();
    wheel[kR].enc_update_speed();

    double positions[2] = {wheel[kL].enc->get_position(), wheel[kR].enc->get_position()};
    interface->wheel_position_cb(positions, 2);

    return true;
};


}
