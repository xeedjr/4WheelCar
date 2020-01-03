
#include <QMC5883L.h>
#include <MPU6050_tockn.h>
#include <Wire.h>
#include "ros_topics.h"
#include "ros_msg/sensor_msgs/Imu.h"
#include <ros.h>
#include <Adafruit_HMC5883_U.h>

/* Assign a unique ID to this sensor at the same time */
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);

sensor_msgs::Imu imu_msg;
ros::Publisher pub_imu( TOPIC_IMU_PUB, &imu_msg);

void imu_setup() {
  Wire.begin();
  /* Initialise the sensor */
  if(!mag.begin())
  {
    /* There was a problem detecting the HMC5883 ... check your connections */
    Serial.println("Ooops, no HMC5883 detected ... Check your wiring!");
    while(1);
  }

  delay(500);
}

void imu_loop() {

}
