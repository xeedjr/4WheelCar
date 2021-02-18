/*
 * Rosserial.cpp
 *
 *  Created on: 30 лист. 2020 р.
 *      Author: Bogdan
 */
#include <math.h>
#include <Rosserial.h>
#include "usbd_cdc_if.h"

#include "geometry_msgs/TransformStamped.h"
#include <tf/tf.h>


namespace ros_serial {

class NewHardware : public ArduinoHardware
{
  public:
  NewHardware():ArduinoHardware(&huart6){};

  void write(uint8_t* data, int length) {
      CDC_Transmit_FS(data, length);
  };
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

    odom_pub = new ros::Publisher( "odom", &odom_msg);
    nh->advertise(*odom_pub);

    /// Subscriptions

    sub_cmd_vel = new ros::Subscriber<geometry_msgs::Twist>("/joy_teleop/cmd_vel", [this_local](const geometry_msgs::Twist& msg){
        this_local->sub_cmd_vel_cb(msg);
    });
    nh->subscribe(*sub_cmd_vel);

    broadcaster.init(*(ros::NodeHandle*)(nh));       // set up broadcaster
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

bool Rosserial::wheel_position_updateV(const QP::QEvt *e) {
    auto ev = (Event*)e;

//    if (nh->connected() == false)
//        return false;

    double vx = 0.1;
    double vy = -0.1;
    double vth = 0.1;
    static double x = 0.0;
    static double y = 0.0;
    static double th = 0.0;
    static ros::Time current_time = nh->now(), last_time = nh->now();
    const float R_wheel = 0.034; ///M radius wheel
    const float R_base = 0.2; ///
    static float prev_l_rad = ev->u[0].f;
    static float prev_r_rad = ev->u[1].f;


    current_time = nh->now();
    double dt = (current_time - last_time).toSec();

    auto curr_l_rad = ev->u[0].f;
    auto curr_r_rad = ev->u[1].f;
    /// s=r*rad
    auto Dl_m = ((curr_l_rad - prev_l_rad) * R_wheel);
    auto Dr_m = ((curr_r_rad - prev_r_rad) * R_wheel);
    auto Dc_m = (Dl_m+Dr_m)/2.0;

    auto prevx = x;
    auto prevy = y;
    auto prevth = th;

    x = x + Dc_m*cos(th);
    y = y + Dc_m*sin(th);
    th = th + (Dr_m - Dl_m)/(R_base*2);

    vx = (x - prevx) * dt;
    vy = (y - prevy) * dt;
    vth = (th - prevth) * dt;

    //since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionFromYaw(th);

    //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    //send the transform
    broadcaster.sendTransform(odom_trans);

    //next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";

    //set the position
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    //set the velocity
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.angular.z = vth;

    //publish the message
    odom_pub->publish(&odom);

    last_time = current_time;
    prev_l_rad = curr_l_rad;
    prev_r_rad = curr_r_rad;

}

/*
bool Rosserial::wheel_position_updateV(const QP::QEvt *e) {
    auto ev = (Event*)e;

    if (nh->connected() == false)
        return false;

    static char base_link[] = "/base_link";
    static char odom[] = "/odom";
    static double x = 0;
    static double y = 0;
    static double theta = 0;
    static long pos0_old = 0;
    static long pos1_old = 0;
    static long pos0_diff = 0;
    static long pos1_diff = 0;
    static float pos0_mm_diff = 0;
    static float pos1_mm_diff = 0;
    static float pos_average_mm_diff = 0;
    static float pos_total_mm = 0;
    static float pos0_mm_prev = 0;
    static float pos1_mm_prev = 0;

    const float R_wheel = 0.034; ///M radius wheel
    const float R_base = 0.2; ///
    /// s=r*rad
    auto pos0_mm = (ev->u[0].f * R_wheel) * 1000.0;
    auto pos1_mm = (ev->u[1].f * R_wheel) * 1000.0;

    // calc mm from encoder counts
    pos0_mm_diff = pos0_mm * pos0_mm_prev;
    pos1_mm_diff = pos1_mm * pos1_mm_prev;

    pos0_mm_prev = pos0_mm;
    pos1_mm_prev = pos1_mm;

    // calc distance travelled based on average of both wheels
    pos_average_mm_diff = (pos0_mm_diff + pos1_mm_diff) / 2;   // difference in each cycle
    pos_total_mm += pos_average_mm_diff;                       // calc total running total distance

    // calc angle or rotation to broadcast with tf
    float phi = ((pos1_mm_diff - pos0_mm_diff) / ((R_base*2)*1000));

    theta += phi;

    if (theta >= (M_PI*2.0)) {
        theta -= (M_PI*2.0);
    }
    if (theta <= (-(M_PI*2.0))) {
        theta += (M_PI*2.0);
    }

    // calc x and y to broadcast with tf

    y += pos_average_mm_diff * sin(theta);
    x += pos_average_mm_diff * cos(theta);

    // *** broadcast odom->base_link transform with tf ***

    geometry_msgs::TransformStamped t;

    t.header.frame_id = odom;
    t.child_frame_id = base_link;

    t.transform.translation.x = x/1000;   // convert to metres
    t.transform.translation.y = y/1000;
    t.transform.translation.z = 0;

    t.transform.rotation = tf::createQuaternionFromYaw(theta);
    t.header.stamp = nh->now();

    broadcaster.sendTransform(t);

    // *** broadcast odom message ***

    nav_msgs::Odometry odom_msg;
    odom_msg.header.stamp = nh->now();
    odom_msg.header.frame_id = odom;
    odom_msg.pose.pose.position.x = x/1000;
    odom_msg.pose.pose.position.y = y/1000;
    odom_msg.pose.pose.position.z = 0.0;
    odom_msg.pose.pose.orientation = tf::createQuaternionFromYaw(theta);

    odom_msg.child_frame_id = base_link;
    odom_msg.twist.twist.linear.x = ((pos0_mm_diff + pos1_mm_diff) / 2)/10;          // forward linear velovity
    odom_msg.twist.twist.linear.y = 0.0;                                        // robot does not move sideways
    odom_msg.twist.twist.angular.z = ((pos1_mm_diff - pos0_mm_diff) / ((R_base*2)*1000))*100;      // anglular velocity

    odom_pub->publish(&odom_msg);
}
*/

bool Rosserial::timer1(const QP::QEvt *e) {
    static uint16_t range = 0;

    if (nh->connected() == false)
        return false;
/*
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

    if (nh->connected() == false)
        return false;
/*
    range_msg_fl.range = ev->u[0].f;
    range_msg_fl.header.stamp = nh->now();
    pub_range_fl->publish(&range_msg_fl);

    range_msg_fr.range = ev->u[1].f;
    range_msg_fr.header.stamp = nh->now();
    pub_range_fr->publish(&range_msg_fr);
    */
}

bool Rosserial::motor_pubV(const QP::QEvt *e) {
    auto ev = (Event*)e;

    if (nh->connected() == false)
        return false;
/*
    motor_msg.left_motor_velocity = ev->u[0].f;
    motor_msg.right_motor_velocity = ev->u[1].f;
    pub_motor->publish(&motor_msg);
    */
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
    if (num < 2)
        return;

    auto ev = (Event*)Q_NEW(QP::QEvt, WHEEL_POSITION_UPDATE_SIG);
    ev->u[0].f = data[0];
    ev->u[1].f = data[1];
    POST(ev, this);
}

}


void roserial_update(uint8_t ch) {
    ros_serial::roserial_update1(ch);
}



