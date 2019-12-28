/*
  Blink
  Turns on an LED on for one second, then off for one second, repeatedly.
 
  This example code is in the public domain.
 */
#include <timer.h>
#include <HCSR04.h>
#include <ros.h>
#include <ros/time.h>
#include "board.h"
#include "ros_topics.h"
#include "ros_msg/sensor_msgs/Range.h"
#include "ros_msg/carmen_msgs/FirmwareCommandWrite.h"
#include "ros_msg/carmen_msgs/FirmwareStateRead.h"
#include "motor.h"
#include "VelocityEncoder.h"

class VelocityEncoder left_ve(ENC1);

auto timer = timer_create_default();
UltraSonicDistanceSensor distanceSensorFL(USFL_TRIG, USFL_ECHO);  // Initialize sensor that uses digital pins 13 and 12.
UltraSonicDistanceSensor distanceSensorFR(USFR_TRIG, USFR_ECHO);  // Initialize sensor that uses digital pins 13 and 12.

class NewHardware : public ArduinoHardware
{
  public:
  NewHardware():ArduinoHardware(&Serial1, 115200){};
};
ros::NodeHandle_<NewHardware>  nh;

/**************** ROS ********************/
/// rostopic pub /motor_sub carmen_msgs/FirmwareCommandWrite "{right_motor_velocity_command : 0}"
void servo_cb( const carmen_msgs::FirmwareCommandWrite& cmd_msg){
  Set_MotorLeft_RadialSpeed(cmd_msg.left_motor_velocity_command);
  Set_MotorRight_RadialSpeed(cmd_msg.right_motor_velocity_command);
  Serial.println("Receive Motor cmd");
  Serial.println(cmd_msg.left_motor_velocity_command);
  Serial.println(cmd_msg.right_motor_velocity_command);
}

// Subs
ros::Subscriber<carmen_msgs::FirmwareCommandWrite> sub(TOPIC_MOTOR_SUB, servo_cb);
// Pub
carmen_msgs::FirmwareStateRead motor_msg;
ros::Publisher pub_range( TOPIC_MOTOR_PUB, &motor_msg);
// Pub
sensor_msgs::Range range_msg_fl;
ros::Publisher pub_range_fl( TOPIC_LEFT_DISTANCE_PUB, &range_msg_fl);
sensor_msgs::Range range_msg_fr;
ros::Publisher pub_range_fr( TOPIC_RIGHT_DISTANCE_PUB, &range_msg_fr);

/*************** ROS ***********************/

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
  
    return true; // to repeat the action - false to stop
}

// the setup routine runs once when you press reset:
void setup() {                
  // initialize the digital pin as an output.
  Serial.begin(115200);
  Serial.println("Hello world");
  nh.initNode();
  //nh.advertise(chatter);
  nh.advertise(pub_range_fl);
  nh.advertise(pub_range_fr);
  nh.subscribe(sub);

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
  
  delay(2000);// Give reader a chance to see the output.
}
 
// the loop routine runs over and over again forever:
void loop() {
  nh.spinOnce();
  timer.tick();
}
