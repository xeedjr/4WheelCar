/*
  Blink
  Turns on an LED on for one second, then off for one second, repeatedly.
 
  This example code is in the public domain.
 */
#include <stdlib.h>
#include <timer.h>
#include <HCSR04.h>
#include <ros.h>
#include <ros/time.h>
#include "board.h"
#include "ros_topics.h"
#include "ros_msg/sensor_msgs/Range.h"
#include "ros_msg/carmen_msgs/FirmwareCommandWrite.h"
#include "ros_msg/carmen_msgs/FirmwareStateRead.h"
#include "ros_msg/sensor_msgs/Imu.h"
#include "motor.h"
#include "VelocityEncoder.h"
#include "ESP8266ATHardware.h"
#include "wifi.h"
#include "imu.h"

class WIFI wifi_module;

class VelocityEncoder left_ve(ENCL);
class VelocityEncoder right_ve(ENCR);

auto timer = timer_create_default();
UltraSonicDistanceSensor distanceSensorFL(USFL_TRIG, USFL_ECHO);  // Initialize sensor that uses digital pins 13 and 12.
UltraSonicDistanceSensor distanceSensorFR(USFR_TRIG, USFR_ECHO);  // Initialize sensor that uses digital pins 13 and 12.

class NewHardware : public ESP8266ATHardware
{
  public:
  NewHardware():ESP8266ATHardware(){};
};
/*class NewHardware : public ArduinoHardware
{
  public:
  NewHardware():ArduinoHardware(&Serial1, 115200){};
};*/

ros::NodeHandle_<NewHardware>  nh;

/**************** ROS ********************/
/// rostopic pub /motor_sub carmen_msgs/FirmwareCommandWrite "{right_motor_velocity_command : 2, left_motor_velocity_command : -2}"
void servo_cb( const carmen_msgs::FirmwareCommandWrite& cmd_msg){
  Set_MotorLeft_RadialSpeed(cmd_msg.left_motor_velocity_command);
  Set_MotorRight_RadialSpeed(cmd_msg.right_motor_velocity_command);
  Serial.println("Receive Motor cmd");
  Serial.println(cmd_msg.left_motor_velocity_command);
  Serial.println(cmd_msg.right_motor_velocity_command);
}

// Subs
ros::Subscriber<carmen_msgs::FirmwareCommandWrite> sub_motor(TOPIC_MOTOR_SUB, servo_cb);
// Pub
carmen_msgs::FirmwareStateRead motor_msg;
ros::Publisher pub_motor( TOPIC_MOTOR_PUB, &motor_msg);
// Pub
sensor_msgs::Range range_msg_fl;
ros::Publisher pub_range_fl( TOPIC_LEFT_DISTANCE_PUB, &range_msg_fl);
sensor_msgs::Range range_msg_fr;
ros::Publisher pub_range_fr( TOPIC_RIGHT_DISTANCE_PUB, &range_msg_fr);
sensor_msgs::Imu imu_msg;
ros::Publisher pub_imu( TOPIC_IMU_PUB, &imu_msg);

/*************** ROS ***********************/
void publish_imu(float q0, float q1, float q2, float q3) {
  imu_msg.header.stamp = nh.now();
  imu_msg.header.frame_id =  TOPIC_IMU_PUB;
  imu_msg.orientation.x = q0;
  imu_msg.orientation.y = q1;
  imu_msg.orientation.z = q2;
  imu_msg.orientation.w = q3;
  pub_imu.publish(&imu_msg);
}


bool function_to_call(void *argument /* optional argument given to in/at/every */) {
    //************** US Left */
    range_msg_fl.range = distanceSensorFL.measureDistanceCm();
    range_msg_fl.header.stamp = nh.now();
    pub_range_fl.publish(&range_msg_fl);
    /*************/
    //************** US Right */
    range_msg_fr.range = distanceSensorFR.measureDistanceCm();
    range_msg_fr.header.stamp = nh.now();
    pub_range_fr.publish(&range_msg_fr);
    /*************/

    nh.loginfo("Program info");

 //   Serial.println(String(left_ve.getSpeed()) + ", " + String(right_ve.getSpeed()));
    motor_msg.left_motor_velocity = left_ve.getSpeed();
    motor_msg.right_motor_velocity = right_ve.getSpeed();
    pub_motor.publish(&motor_msg);

    return true; // to repeat the action - false to stop
}

// the setup routine runs once when you press reset:
void setup() {                
  // initialize the digital pin as an output.
  Serial.begin(115200);
  Serial.println("Hello world");
  wifi_module.setup();
    
  nh.initNode();

  nh.advertise(pub_range_fl);
  nh.advertise(pub_range_fr);
  nh.subscribe(sub_motor);
  nh.advertise(pub_motor);
  nh.advertise(pub_imu);
  
  //************ US Left */
  range_msg_fl.radiation_type = sensor_msgs::Range::ULTRASOUND;
  range_msg_fl.header.frame_id =  TOPIC_LEFT_DISTANCE_PUB;
  range_msg_fl.field_of_view = 0.1;  // fake
  range_msg_fl.min_range = -1.0;
  range_msg_fl.max_range = 400.0;
  /************* */
  //************ US Right */
  range_msg_fr.radiation_type = sensor_msgs::Range::ULTRASOUND;
  range_msg_fr.header.frame_id =  TOPIC_RIGHT_DISTANCE_PUB;
  range_msg_fr.field_of_view = 0.1;  // fake
  range_msg_fr.min_range = -1.0;
  range_msg_fr.max_range = 400.0;
  /************* */

  timer.every(1000, function_to_call);
  imu_setup();
  delay(2000);// Give reader a chance to see the output.
}
 
// the loop routine runs over and over again forever:
void loop() {
  if (auto ret = nh.spinOnce() != 0) 
    Serial.println("error " + String(ret));
  timer.tick();
  imu_loop();
  wifi_module.loop();

}
