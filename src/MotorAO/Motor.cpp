/*
 * Motor.cpp
 *
 *  Created on: 30 лист. 2020 р.
 *      Author: Bogdan
 */
#include <math.h>
#include <Motor.h>

volatile float speed = 3.0;
volatile double p = 0;
volatile double i = 0;
volatile double d = 0;
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

    wheel[kL].pid = new MiniPID(10, 2, 2);
    wheel[kL].pid->setOutputLimits(0, 100.0);

    setAttr(QP::TASK_NAME_ATTR, "Motor");
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
    TB6612FNG::Mode L_direction;
    TB6612FNG::Mode R_direction;

    wheel[kL].pid_update();
    wheel[kR].pid_update();

    if (wheel[kL].is_reverse) {
        if (wheel[kL].target_wheel_speed > 0) {
            L_direction = TB6612FNG::Mode::kCCW;
        } else {
            L_direction = TB6612FNG::Mode::kCW;
        }
    } else {
        if (wheel[kL].target_wheel_speed > 0) {
            L_direction = TB6612FNG::Mode::kCW;
        } else {
            L_direction = TB6612FNG::Mode::kCCW;
        }
    }
    if (wheel[kR].is_reverse) {
        if (wheel[kR].target_wheel_speed > 0) {
            R_direction = TB6612FNG::Mode::kCCW;
        } else {
            R_direction = TB6612FNG::Mode::kCW;
        }
    } else {
        if (wheel[kR].target_wheel_speed > 0) {
            R_direction = TB6612FNG::Mode::kCW;
        } else {
            R_direction = TB6612FNG::Mode::kCCW;
        }
    }

/*    wheel[kL].pwm_total = wheel[kL].pwm_based_on_targed + wheel[kL].pwm;
    wheel[kR].pwm_total = wheel[kR].pwm_based_on_targed + wheel[kR].pwm;
    if (wheel[kL].pwm_total > 100)
        wheel[kL].pwm_total = 100;
    if (wheel[kR].pwm_total > 100)
        wheel[kR].pwm_total = 100;
    if (wheel[kL].pwm_total < 0)
        wheel[kL].pwm_total = 0;
    if (wheel[kR].pwm_total < 0)
        wheel[kR].pwm_total = 0;
*/
    drive->drive(TB6612FNG::Channels::kB, L_direction, wheel[kL].pwm);
    drive->drive(TB6612FNG::Channels::kA, R_direction, wheel[kR].pwm);

    printf("%f %f %f\n",
            std::abs(wheel[kL].target_wheel_speed),
            std::abs(wheel[kL].current_wheel_speed),
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
