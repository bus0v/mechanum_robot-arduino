#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Imu.h>

//half width in cm
float d1 = 0.1070;
// half length in cm
float d2 = 0.0830;
// wheel radius in cm
float R = 0.0400;
float sum_d = d1+d2;
float vx,vy,wz;
ros::NodeHandle nh;
ros::Subscriber<std_msgs::Twist> sub("twist", &messageCb);
// no need ofr targetpid
sensor_msgs::Imu imu_msg;
//recieve twist message
// get v and w out of the message
void messageCb(const std_msgs::Twist &twist_msg){

 v_plan_x = twist_msg.linear.x;
 v_plan_y = twist_msg.linear.y;
 w_plan = twist_msg.angular.z;

 }

//pid on v_x
PID pid_vx = PID(&v_backx, &vx, &v_plan_x,5,2.5,0.5,DIRECT));
//pid on v y
PID pid_vy = PID(&v_backy, &vy, &v_plan_y,5,2.5,0.5,DIRECT));
//pid on w
PID pid_w = PID(&wz_rad, &wz, &w_plan_x,5,2.5,0.5,DIRECT));

// inverse kinematics on the v and w
pwr[1] =(1/R) * (vx -vy -(sumd *wz));
pwr[0] =(1/R) * (vx +vy +(sumd *wz));
pwr[2] =(1/R) * (vx +vy -(sumd *wz));
pwr[3] =(1/R) * (vx -vy +(sumd *wz));

for(int k = 0; k < 4; k++){
setMotor(dir,pwr[k],k);}
// compute speeds

// filter encoder
//get vFilt
// forward Kinematics
v_backx = (R/4)*(vFilt[0] + vFilt[1] + vFilt[2] + vFilt[3])
v_backy = (R/4)*(vFilt[0] - vFilt[1] + vFilt[2] - vFilt[3])
// get gyroscope reading for w
IMU.getGyro(&gyroData);
wz_rad = gyroData.gyroZ * 0.0174533
// pass update to pid
