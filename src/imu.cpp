
#include "MPU9250.h"
#include <MadgwickAHRS.h>

Madgwick filter;
MPU9250  IMU(*BMP280_I2C_DRIVER, 0x68);
int status;
unsigned long msPerReading, msPrevious;
extern void publish_imu(float q0, float q1, float q2, float q3);

void imu_setup() {
  // start communication with IMU 
  status = IMU.begin();
  if (status < 0) {
    while(1) {}
  }
  
  // setting the accelerometer full scale range to +/-8G 
  IMU.setAccelRange(MPU9250::ACCEL_RANGE_8G);
  // setting the gyroscope full scale range to +/-500 deg/s
  IMU.setGyroRange(MPU9250::GYRO_RANGE_500DPS);
  // setting DLPF bandwidth to 20 Hz
  IMU.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_20HZ);
  // setting SRD to 19 for a 50 Hz update rate
  IMU.setSrd(19);  
  
  filter.begin(20);
  // initialize variables to pace updates to correct rate
  msPerReading = 1000 / 20;
  msPrevious = TIME_I2MS(osalOsGetSystemTimeX());
}

void imu_loop() {
  int aix, aiy, aiz;
  int gix, giy, giz;
  float ax, ay, az;
  float gx, gy, gz;
  static float roll, pitch, heading;
  unsigned long msNow;
  
  // check if it's time to read data and update the filter
  msNow = TIME_I2MS(osalOsGetSystemTimeX());
  if (msNow - msPrevious >= msPerReading) {
  
    // read the sensor
    IMU.readSensor();
  
    // update the filter, which computes orientation
    filter.update(IMU.getGyroX_rads()/0.0174533f, IMU.getGyroY_rads()/0.0174533f, IMU.getGyroZ_rads()/0.0174533f,
                  IMU.getAccelX_mss(), IMU.getAccelY_mss(), IMU.getAccelZ_mss(),  
                  IMU.getMagX_uT(), IMU.getMagY_uT(), IMU.getMagZ_uT());

//    publish_imu(filter.q0, filter.q1, filter.q2, filter.q3);
    
    // print the heading, pitch and roll
    roll = filter.getRoll();
    pitch = filter.getPitch();
    heading = filter.getYaw();
//    Serial.print("Orientation: ");
//    Serial.print(heading);
//    Serial.print(" ");
//    Serial.print(pitch);
//    Serial.print(" ");
//    Serial.println(roll);

    // increment previous time, so we keep proper pace
    msPrevious = msPrevious + msPerReading;
  }  
  
}
