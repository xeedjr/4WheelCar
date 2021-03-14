/*
 * Motor.cpp
 *
 *  Created on: 30 лист. 2020 р.
 *      Author: Bogdan
 */
#include <math.h>
#include <algorithm>
#include <Motor.h>

volatile float speed = 3.0;
volatile double p = 10;
volatile double i = 2;
volatile double d = 2;
volatile double p_pre = 0.0;
volatile double i_pre = 0.0;
volatile double d_pre = 0.0;

namespace motor {

Motor::Motor(TB6612FNG *drive, WheelMotorEncoder *enc1, WheelMotorEncoder *enc2, MotorInterface *interface) :
	drive(drive),
	interface(interface)
{
    wheel[kL].enc = enc1;
    wheel[kR].enc = enc2;

    wheel[kL].is_reverse = true;

    wheel[kR].pid = new MiniPID(10, 2, 2);
    wheel[kR].pid->setOutputLimits(-300.0, 300.0);

    wheel[kL].pid = new MiniPID(10, 2, 2);
    wheel[kL].pid->setOutputLimits(-300.0, 300.0);

    setAttr(QP::TASK_NAME_ATTR, "Motor");
}

Motor::~Motor() {
	// TODO Auto-generated destructor stub
}

bool Motor::set_speed_left(const QP::QEvt *e) {
    auto ev = (Event*)e;

    wheel[kL].target_wheel_speed = std::max((float)-20.0, std::min(ev->u[0].f, (float)20));

    return true;
};

bool Motor::set_speed_right(const QP::QEvt *e) {
    auto ev = (Event*)e;

    wheel[kR].target_wheel_speed = std::max((float)-20.0, std::min(ev->u[0].f, (float)20));

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
    TB6612FNG::Mode L_direction;
    TB6612FNG::Mode R_direction;

    wheel[kL].pid_update();
    wheel[kR].pid_update();

    wheel[kR].pwm = std::max((float)-100.0, std::min(wheel[kR].pwm, (float)100.0));
    wheel[kL].pwm = std::max((float)-100.0, std::min(wheel[kL].pwm, (float)100.0));

    if (wheel[kL].is_reverse) {
        if (wheel[kL].pwm > 0) {
            L_direction = TB6612FNG::Mode::kCCW;
        } else {
            L_direction = TB6612FNG::Mode::kCW;
        }
    } else {
        if (wheel[kL].pwm > 0) {
            L_direction = TB6612FNG::Mode::kCW;
        } else {
            L_direction = TB6612FNG::Mode::kCCW;
        }
    }
    if (wheel[kR].is_reverse) {
        if (wheel[kR].pwm > 0) {
            R_direction = TB6612FNG::Mode::kCCW;
        } else {
            R_direction = TB6612FNG::Mode::kCW;
        }
    } else {
        if (wheel[kR].pwm > 0) {
            R_direction = TB6612FNG::Mode::kCW;
        } else {
            R_direction = TB6612FNG::Mode::kCCW;
        }
    }

    drive->drive(TB6612FNG::Channels::kB, L_direction, std::abs(wheel[kL].pwm));
    drive->drive(TB6612FNG::Channels::kA, R_direction, std::abs(wheel[kR].pwm));

    printf("%f %f %f\n",
            wheel[kL].target_wheel_speed,
            wheel[kL].current_wheel_speed,
            //wheel[kL].pwm_based_on_targed,
            wheel[kL].pwm);
/*
    if (wheel[kL].target_wheel_speed != speed) {
        SetSpeedL(speed);
    }
    if (p != p_pre) {
        wheel[kL].pid->setP(p);
        p_pre = p;
    }
    if (i != i_pre) {
        wheel[kL].pid->setI(i);
        i_pre = i;
    }
    if (d != d_pre) {
        wheel[kL].pid->setD(d);
        d_pre = d;
    }
*/
    //printf("%f %f\n", wheel[kL].pwm_based_on_targed, wheel[kR].pwm_based_on_targed)

    return true;
};

bool Motor::get_wheel_speed(const QP::QEvt *e) {
    auto ev = (Event*)e;

    /// perios 100 ms

    wheel[kL].enc_update_speed();
    wheel[kR].enc_update_speed();

   // printf("%f, %f\n", wheel[kL].current_wheel_speed, wheel[kR].current_wheel_speed);

    double positions[2] = {wheel[kL].current_wheel_pos, wheel[kR].current_wheel_pos};
    interface->wheel_position_cb(positions, 2);

    double speeds[2] = {wheel[kL].current_wheel_speed, wheel[kR].current_wheel_speed};
    interface->wheel_curr_speed_cb(speeds, 2);

    return true;
};


}
