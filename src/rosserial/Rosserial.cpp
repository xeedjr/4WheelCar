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

void roserial_update1(uint8_t ch) {
    if (!this_local->recv_is_enabled)
        return;

    auto ev = (Event*)Q_NEW_FROM_ISR(Event, RECEIVED_BYTE_SIG);
    ev->u[0].u8 = ch;
    this_local->POST_FROM_ISR(ev, nullptr, this);
}

Rosserial::Rosserial(TIM_HandleTypeDef *htim) :
        htim(htim)
{
    this_local = this;
    setAttr(QP::TASK_NAME_ATTR, "Rosserial");
}

Rosserial::~Rosserial() {
	// TODO Auto-generated destructor stub
}

bool Rosserial::initialize(const QP::QEvt *e) {
/*    if (HAL_TIM_RegisterCallback(htim,
            HAL_TIM_PERIOD_ELAPSED_CB_ID,
            [this_local](TIM_HandleTypeDef *htim){
                this_local->
            }) != HAL_OK)
        exit(-1);

    if(HAL_TIM_Base_Start_IT(htim) != HAL_OK)
        exit(-1);
*/
    nh = new ros::NodeHandle_<NewHardware>;
    if (nh == nullptr)
        exit(-1);

    nh->initNode();

    /// Publishers

    pub_range_fl = new ros::Publisher( "/car/sonar/left", &range_msg_fl);
    nh->advertise(*pub_range_fl);

    pub_range_fr = new ros::Publisher( "/car/right/left", &range_msg_fr);
    nh->advertise(*pub_range_fr);

    pub_motor = new ros::Publisher( "/car/wheels/odometry", &motor_msg);
    nh->advertise(*pub_motor);

    /// Subscriptions

    sub_cmd_vel = new ros::Subscriber<geometry_msgs::Twist>("/joy_teleop/cmd_vel", [this_local](const geometry_msgs::Twist& msg){
        this_local->sub_cmd_vel_cb(msg);
    });
    nh->subscribe(*sub_cmd_vel);

}

bool Rosserial::process_in_data(const QP::QEvt *e) {
    auto ev = (Event*)e;

    nh->getHardware()->rbyte = ev->u[0].u8;
    auto ret = nh->spinOnce();
}

bool Rosserial::spin_data(const QP::QEvt *e) {
    auto ev = (Event*)e;
    auto ret = nh->spinOnce();
}

bool Rosserial::timer1(const QP::QEvt *e) {
    static uint16_t range = 0;
    range_msg_fl.range = range++;
    range_msg_fl.header.stamp = nh->now();
    pub_range_fl->publish(&range_msg_fl);

    range_msg_fr.range = range;
    range_msg_fr.header.stamp = nh->now();
    pub_range_fr->publish(&range_msg_fr);

    motor_msg.left_motor_velocity = +125;
    motor_msg.right_motor_velocity = -100;
    pub_motor->publish(&motor_msg);

}

void Rosserial::sub_cmd_vel_cb(const geometry_msgs::Twist& msg) {
    const float R_wheel = 0.034; ///M radius wheel
    const float R_base = 0.2; ///
    /// s=r*rad
    float rad_per_sec_linear = (msg.linear.x) / (R_wheel);   ///rad = s / R

    float m_per_sec_ang = (R_base) * (msg.angular.z);   ///s=r*rad
    float rad_per_sec_ang =  (m_per_sec_ang) / (R_wheel);   ///s=r*rad

    //printf("%f %f\n", (rad_per_sec_linear - rad_per_sec_ang), rad_per_sec_linear + rad_per_sec_ang);

    motor->SetSpeedL(rad_per_sec_linear - rad_per_sec_ang);
    motor->SetSpeedR(rad_per_sec_linear + rad_per_sec_ang);
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


/// public interfaces
void Rosserial::update_Sensors_cb(float y, float p, float r)
{

}

void Rosserial::us_sensor_cb(float* data, uint8_t num) {

}

void Rosserial::tof_sensors_cb(float* data, uint8_t num) {

}

void Rosserial::wheel_position_cb(double* data, uint8_t num) {

}

}


void roserial_update(uint8_t ch) {
    ros_serial::roserial_update1(ch);
}



