/*
 * Rosserial.h
 *
 *  Created on: 30 лист. 2020 р.
 *      Author: Bogdan
 */

#pragma once

#include <functional>
#include <math.h>       /* fabs */

#include "RosserialQM.h"
#include <ros.h>
#include "sensor_msgs/Range.h"
#include "geometry_msgs/Twist.h"
#include "carmen_msgs/FirmwareCommandWrite.h"
#include "carmen_msgs/FirmwareStateRead.h"
#include "Motor.h"

namespace ros_serial {

struct Event : public QP::QEvt {
    union U {
         float f;
         void *p;
         uint32_t *u32p;
         uint64_t *u64p;
         char *str;
         uint8_t u8;
         uint32_t u32;
         uint64_t u64;
         bool b;
         U() {};
         ~U() {};
    } u[3];

    Event(QP::QSignal const s) : QEvt(s) {};
};

class Rosserial : public RosserialQM {

    sensor_msgs::Range range_msg_fl;
    ros::Publisher *pub_range_fl;
    sensor_msgs::Range range_msg_fr;
    ros::Publisher *pub_range_fr;
    carmen_msgs::FirmwareStateRead motor_msg;
    ros::Publisher *pub_motor;

    ros::Subscriber<geometry_msgs::Vector3> *sub_cmd_vel;

private:
    TIM_HandleTypeDef *htim;
    motor::Motor *motor;
	Event const *active_event  = nullptr;

	uint8_t stack[1024];
    QP::QEvt const *queueSto[127] = {0};

    void sub_cmd_vel_cb(const geometry_msgs::Vector3& msg);

    bool initialize(const QP::QEvt *e);
    bool process_in_data(const QP::QEvt *e);
    bool timer1(const QP::QEvt *e);
    bool sonar_pubV(const QP::QEvt *e);
    bool motor_pubV(const QP::QEvt *e);
    bool imu_pubV(const QP::QEvt *e);
    bool spin_data(const QP::QEvt *e);

public:
	Rosserial(TIM_HandleTypeDef *htim, motor::Motor *motor);
	virtual ~Rosserial();

	void startAO(){
	    start(8U, // priority
	                 queueSto, Q_DIM(queueSto),
	#ifndef WIN32
	                 stack, sizeof(stack)); // no stack
	#else
	                 nullptr, 0); // no stack
	#endif
	}

    void imu_pub(float r, float p, float y) {
        auto ev = (Event*)Q_NEW(QP::QEvt, IMU_PUBLISH_SIG);
        ev->u[0].f = r;
        ev->u[1].f = p;
        ev->u[1].f = y;
        POST(ev, this);
    }
	void sonar_pub(float left, float right) {
	    auto ev = (Event*)Q_NEW(QP::QEvt, SONAR_PUBLISH_SIG);
	    ev->u[0].f = left;
	    ev->u[1].f = right;
	    POST(ev, this);
	}
    void motor_pub(float left, float right) {
        auto ev = (Event*)Q_NEW(QP::QEvt, MOTOR_PUBLISH_SIG);
        ev->u[0].f = left;
        ev->u[1].f = right;
        POST(ev, this);
    }

};
};
