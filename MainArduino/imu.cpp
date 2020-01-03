
#include <QMC5883L.h>
#include <MPU6050_tockn.h>
#include <Wire.h>
#include "ros_topics.h"
#include "ros_msg/sensor_msgs/Imu.h"
#include <ros.h>
#include <DFRobot_QMC5883.h>

//https://osoyoo.com/driver/DFRobot_QMC5883.zip

DFRobot_QMC5883 compass;

sensor_msgs::Imu imu_msg;
ros::Publisher pub_imu( TOPIC_IMU_PUB, &imu_msg);

void publish() {
  imu_msg.header.stamp = nh.now();
  imu_msg.header.frame_id =  TOPIC_IMU_PUB;
}

void imu_setup() {
  while (!compass.begin())
  {
    Serial.println("Could not find a valid QMC5883 sensor, check wiring!");
    delay(500);
  }

    if(compass.isHMC()){
        Serial.println("Initialize HMC5883");
        compass.setRange(HMC5883L_RANGE_1_3GA);
        compass.setMeasurementMode(HMC5883L_CONTINOUS);
        compass.setDataRate(HMC5883L_DATARATE_15HZ);
        compass.setSamples(HMC5883L_SAMPLES_8);
    }
   else if(compass.isQMC()){
        Serial.println("Initialize QMC5883");
        compass.setRange(QMC5883_RANGE_2GA);
        compass.setMeasurementMode(QMC5883_CONTINOUS); 
        compass.setDataRate(QMC5883_DATARATE_50HZ);
        compass.setSamples(QMC5883_SAMPLES_8);
   }
}

void imu_loop() {
  Vector mag = compass.readRaw();
  Serial.print(mag.XAxis);
  Serial.print(":");
  Serial.print(mag.YAxis);
  Serial.print(":");
  Serial.println(mag.ZAxis);
}
