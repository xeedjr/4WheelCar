/*
  Blink
  Turns on an LED on for one second, then off for one second, repeatedly.
 
  This example code is in the public domain.
 */
#include <HCSR04.h>
#include <ros.h>
#include "board.h"
#include "ros_topics.h"
#include "ros_msg/carmen_msgs/FirmwareCommandWrite.h"
#include "ros_msg/carmen_msgs/FirmwareStateRead.h"
#include "motor.h"

ros::NodeHandle  nh;
UltraSonicDistanceSensor distanceSensor1(US1_TRIG, US1_ECHO);  // Initialize sensor that uses digital pins 13 and 12.
UltraSonicDistanceSensor distanceSensor2(US2_TRIG, US2_ECHO);  // Initialize sensor that uses digital pins 13 and 12.

/**************** ROS ********************/
void servo_cb( const carmen_msgs::FirmwareCommandWrite& cmd_msg){
  Set_MotorLeft_RadialSpeed(cmd_msg.motor_1_velocity_command);
  Set_MotorLeft_RadialSpeed(cmd_msg.motor_2_velocity_command);
}

// Subs
ros::Subscriber<carmen_msgs::FirmwareCommandWrite> sub(TOPIC_MOTOR_SUB, servo_cb);
// Pub
carmen_msgs::FirmwareStateRead motor_msg;
ros::Publisher pub_range( TOPIC_MOTOR_PUB, &motor_msg);
// Pub
sensor_msgs::Range range_msg_fl;
ros::Publisher pub_range( TOPIC_DISTANCE_PUB, &range_msg_fl);
sensor_msgs::Range range_msg_fr;
ros::Publisher pub_range( TOPIC_DISTANCE_PUB, &range_msg_fr);

/*************** ROS ***********************/


// the setup routine runs once when you press reset:
void setup() {                
  // initialize the digital pin as an output.
  Serial.begin(115200);
  Serial.println("Hello world");
  nh.initNode();
  nh.subscribe(sub);

  //************ US Left */
  range_msg_fl.radiation_type = sensor_msgs::Range::ULTRASOUND;
  range_msg_fl.header.frame_id =  "FL";
  range_msg_fl.field_of_view = 0.1;  // fake
  range_msg_fl.min_range = -1.0;
  range_msg_fl.max_range = 400.0;
  /************* */
  //************ US Right */
  range_msg_fr.radiation_type = sensor_msgs::Range::ULTRASOUND;
  range_msg_fr.header.frame_id =  "FR";
  range_msg_fr.field_of_view = 0.1;  // fake
  range_msg_fr.min_range = -1.0;
  range_msg_fr.max_range = 400.0;
  /************* */
  delay(2000);// Give reader a chance to see the output.
}
 
// the loop routine runs over and over again forever:
void loop() {

  //************** US Left */
  range_msg_fl.range = distanceSensor1.measureDistanceCm();
  range_msg_fl.header.stamp = nh.now();
  pub_range_fl.publish(&range_msg);
  /*************/
  //************** US Right */
  range_msg_fr.range = distanceSensor2.measureDistanceCm();
  range_msg_fr.header.stamp = nh.now();
  pub_range_fr.publish(&range_msg);
  /*************/
  
  nh.spinOnce();
}
