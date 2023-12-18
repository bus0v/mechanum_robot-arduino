#include "FastIMU.h"
#include <ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Temperature.h>
#define IMU_ADDRESS 0x68
MPU6050 IMU;                //Change "BMI055" to the name of any supported IMU!
// Currently supported IMUS: MPU9255 MPU9250 MPU6500 MPU6050 ICM20689 ICM20690 BMI055 BMX055

calData calib = { 0 };  //Calibration data
AccelData accelData;    //Sensor data
GyroData gyroData;

ros::NodeHandle nh;
sensor_msgs::Imu imu_msg;
sensor_msgs::Temperature temp_msg;
ros::Publisher imu_data("/imu_data", &imu_msg);
ros::Publisher temp_data("/temp", &temp_msg);
void setup() {
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.advertise(imu_data);
  nh.advertise(temp_data);
  calib.accelBias[0] = -0.05;

  calib.accelBias[1] = -0.03;

  calib.accelBias[2] = 0.16;

  calib.gyroBias[0]=-8.24;

  calib.gyroBias[1]=3.26;

  calib.gyroBias[2] = -3.68;
  calib.valid = 1;
  int err = IMU.init(calib, IMU_ADDRESS);

  err = IMU.setGyroRange(500);      //USE THESE TO SET THE RANGE, IF AN INVALID RANGE IS SET IT WILL RETURN -1
  err = IMU.setAccelRange(2);       //THESE TWO SET THE GYRO RANGE TO ±500 DPS AND THE ACCELEROMETER RANGE TO ±2g

}

void loop() {
  IMU.update();
  IMU.getAccel(&accelData);
  imu_msg.linear_acceleration.x = accelData.accelX;
  
  imu_msg.linear_acceleration.y = accelData.accelY;
  
  imu_msg.linear_acceleration.z = accelData.accelZ;
  
  IMU.getGyro(&gyroData);
  imu_msg.angular_velocity.x = gyroData.gyroX;
  
  imu_msg.angular_velocity.y = gyroData.gyroY;
  
  imu_msg.angular_velocity.z = gyroData.gyroZ;
  
  temp_msg.temperature = IMU.getTemp();
  imu_data.publish(&imu_msg);
  temp_data.publish(&temp_msg);
  nh.spinOnce();
  
}
