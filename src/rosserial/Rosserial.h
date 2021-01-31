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
#include "ros_topics.h"
#include "sensor_msgs/Range.h"
#include "carmen_msgs/FirmwareCommandWrite.h"
#include "carmen_msgs/FirmwareStateRead.h"


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
         std::function<void(bool)> b_cb;
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

    ros::Subscriber<carmen_msgs::FirmwareCommandWrite> *sub_motor;
private:
	Event const *active_event  = nullptr;

	uint8_t stack[1024];
    QP::QEvt const *queueSto[127] = {0};

    void sub_motor_cb(const carmen_msgs::FirmwareCommandWrite& msg);

    bool initialize(const QP::QEvt *e);
    bool process_in_data(const QP::QEvt *e);
    bool timer1(const QP::QEvt *e);

public:
	Rosserial();
	virtual ~Rosserial();

};
};
