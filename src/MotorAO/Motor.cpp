/*
 * Motor.cpp
 *
 *  Created on: 30 лист. 2020 р.
 *      Author: Bogdan
 */
#include <math.h>
#include <Motor.h>

namespace motor {

Motor::Motor(TB6612FNG *drive, RPMEncoderOptical *enc1, RPMEncoderOptical *enc2) :
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

bool Motor::set_speed_left(const QP::QEvt *e) {
    auto ev = (Event*)e;
    float speed = ev->u[0].f;
    float pwm = (100.0/300.0)*fabs(speed);
    TB6612FNG::Mode mode = TB6612FNG::Mode::kCW;
    if (speed > 0) {
        mode = TB6612FNG::Mode::kCCW;
    }
    drive->drive(TB6612FNG::Channels::kA, mode, pwm);
    return true;
};

bool Motor::set_speed_right(const QP::QEvt *e) {
    auto ev = (Event*)e;
    float speed = ev->u[0].f;
    float pwm = (100.0/300.0)*fabs(speed);
    TB6612FNG::Mode mode = TB6612FNG::Mode::kCW;
    if (speed > 0) {
        mode = TB6612FNG::Mode::kCCW;
    }
    drive->drive(TB6612FNG::Channels::kB, mode, pwm);
    return true;
};

bool Motor::pid_init(const QP::QEvt *e) {
    auto ev = (Event*)e;


    pidController.setOutputMin(0);      // This is the default
    pidController.setOutputMax(1023);   // The Arduino has a 10-bit'analog' PWM output,
                                        // but the maximum output can be adjusted down.

    pidController.init(1002);  // Initialize the pid controller to make sure there
                                        // are no output spikes

    return true;
};

bool Motor::pid_timeout(const QP::QEvt *e) {
    auto ev = (Event*)e;

    unsigned short outputValue = pidController.compute(30);   // Compute the PID output

    return true;
};



}
