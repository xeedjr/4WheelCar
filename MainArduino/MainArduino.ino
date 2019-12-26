/*
  Blink
  Turns on an LED on for one second, then off for one second, repeatedly.
 
  This example code is in the public domain.
 */
#include <HCSR04.h>
#include <ros.h>
#include "ros_msg/carmen_msgs/FirmwareCommandWrite.h"
#include "ros_msg/carmen_msgs/FirmwareStateRead.h"
#include "motor.h"

ros::NodeHandle  nh;

/**************** ROS ********************/
void servo_cb( const carmen_msgs::FirmwareCommandWrite& cmd_msg){
  Set_MotorLeft_RadialSpeed(cmd_msg.motor_1_velocity_command);
  Set_MotorLeft_RadialSpeed(cmd_msg.motor_2_velocity_command);
}

ros::Subscriber<carmen_msgs::FirmwareCommandWrite> sub("servo", servo_cb);
carmen_msgs::FirmwareStateRead StateRead;
ros::Publisher pub_range( "/ultrasound", &range_msg);
/*************** ROS ***********************/


// the setup routine runs once when you press reset:
void setup() {                
  // initialize the digital pin as an output.
  Serial.begin(115200);
  Serial.println("Hello world");
  nh.initNode();
  nh.subscribe(sub);
  delay(2000);// Give reader a chance to see the output.
}
 
// the loop routine runs over and over again forever:
void loop() {
  nh.spinOnce();
}
