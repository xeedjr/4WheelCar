/*
 * Rosserial.cpp
 *
 *  Created on: 30 лист. 2020 р.
 *      Author: Bogdan
 */
#include <math.h>
#include <Rosserial.h>

namespace ros_serial {

class NewHardware : public ArduinoHardware
{
  public:
  NewHardware():ArduinoHardware(&huart6){};
};

static ros::NodeHandle_<NewHardware>  *nh;

static class Rosserial *this_local = nullptr;

void roserial_update1() {
    auto ev = (Event*)Q_NEW_FROM_ISR(QP::QEvt, RECEIVED_BYTE_SIG);
    this_local->POST_FROM_ISR(ev, nullptr, this);
}

Rosserial::Rosserial()
{
    this_local = this;
}

Rosserial::~Rosserial() {
	// TODO Auto-generated destructor stub
}

void Rosserial::sub_motor_cb(const carmen_msgs::FirmwareCommandWrite& msg) {
    printf("TT %d", msg.left_motor_p);
}

bool Rosserial::initialize(const QP::QEvt *e) {
    nh = new ros::NodeHandle_<NewHardware>;
    if (nh == nullptr)
        exit(-1);

    nh->initNode();

    pub_range_fl = new ros::Publisher( "/left_sonar", &range_msg_fl);
    nh->advertise(*pub_range_fl);

    pub_range_fr = new ros::Publisher( "/right_sonar", &range_msg_fr);
    nh->advertise(*pub_range_fr);

    pub_motor = new ros::Publisher( "/motor_pub", &motor_msg);
    nh->advertise(*pub_motor);

    sub_motor = new ros::Subscriber<carmen_msgs::FirmwareCommandWrite>("/motor_sub", [this_local](const carmen_msgs::FirmwareCommandWrite& msg){
        this_local->sub_motor_cb(msg);
    });
    nh->subscribe(*sub_motor);
}

bool Rosserial::process_in_data(const QP::QEvt *e) {
    auto ret = nh->spinOnce();
}

bool Rosserial::timer1(const QP::QEvt *e) {
/*    static uint16_t range = 0;
    range_msg_fl.range = range++;
    range_msg_fl.header.stamp = nh->now();
    pub_range_fl->publish(&range_msg_fl);

    range_msg_fr.range = range;
    range_msg_fr.header.stamp = nh->now();
    pub_range_fr->publish(&range_msg_fr);

    motor_msg.left_motor_velocity = +125;
    motor_msg.right_motor_velocity = -100;
    pub_motor->publish(&motor_msg);
*/
}

bool Rosserial::sonar_pubV(const QP::QEvt *e) {
    auto ev = (Event*)e;

    range_msg_fl.range = ev->u[0].f;
    range_msg_fl.header.stamp = nh->now();
    pub_range_fl->publish(&range_msg_fl);

    range_msg_fr.range = ev->u[1].f;
    range_msg_fr.header.stamp = nh->now();
    pub_range_fr->publish(&range_msg_fr);
}

bool Rosserial::motor_pubV(const QP::QEvt *e) {
    auto ev = (Event*)e;
    motor_msg.left_motor_velocity = ev->u[0].f;
    motor_msg.right_motor_velocity = ev->u[1].f;
    pub_motor->publish(&motor_msg);
}

bool Rosserial::imu_pubV(const QP::QEvt *e) {
}


}


void roserial_update() {
    ros_serial::roserial_update1();
}
