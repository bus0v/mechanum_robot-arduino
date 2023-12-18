#include <ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/Twist.h>
#include <filters.h>
#include <PID_v1.h>
#include <std_msgs/Int16MultiArray.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Temperature.h>
#include "motors.h"
#include "ultrasound_sensors.h"
#include "FastIMU.h"
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Temperature.h>
#define IMU_ADDRESS 0x68
MPU6050 IMU;

calData calib = { 0 };  //Calibration data
AccelData accelData;    //Sensor data
GyroData gyroData;

//half width in cm
float d1 = 0.1070;
// half length in cm
float d2 = 0.0830;
// wheel radius in cm
float R = 0.0400;
float sum_d = d1+d2;

//FR,FL,BL,BR
// define pin lists
const int encA[] = {3, 19, 18, 2};
const int encB[] = {26, 15, 10, 5};
const int encoderMin = -32768;
const int encoderMax = 32767;
double targetpid[4] = {0,0,0,0};
volatile int newPosition [] = {0,0,0,0};
volatile float velocity[] = {0,0,0,0};
volatile float distance[] = {0,0,0,0};
volatile int posPrev[] = {0,0,0,0};
long t0 = 0;
float e0 = 0;
float eInt = 0;
float vFilt_float[4];
//filter params
const float cutoff_freq   = 20000.0;  //Cutoff frequency in Hz
const float sampling_time = 0.0000001; //Sampling time in seconds.
IIR::ORDER  order  = IIR::ORDER::OD1; // Order (OD1 to OD4)

// Low-pass filter
Filter f(cutoff_freq, sampling_time, order);
double vx,vy,wz;
double v_backx, v_backy, wz_rad;
double v_plan_x, v_plan_y, w_plan;
double vFilt[] = {0,0,0,0};
double vPrev[] = {0,0,0,0};
double v_cm[] = {0,0,0,0};
double v_rpm[] = {0,0,0,0};
double pwr[] = {0,0,0,0};
int dir;

double kp = 5, ki = 5, kd = 0.1;
//20 5 0.5
//can also do 5 1 0,5??
//FR is weird
//pid on v_x
PID pid_vx = PID(&v_backx, &vx, &v_plan_x,5,2.5,0.5,DIRECT);
//pid on v y
PID pid_vy = PID(&v_backy, &vy, &v_plan_y,5,2.5,0.5,DIRECT);
//pid on w
PID pid_wz = PID(&wz_rad, &wz, &w_plan,5,2.5,0.5,DIRECT);

//instantiate the node handle
// ros::NodeHandle nh;

void messageCb(const geometry_msgs::Twist &twist_msg){
 noInterrupts();
 v_plan_x = twist_msg.linear.x;
 v_plan_y = twist_msg.linear.y;
 w_plan = twist_msg.angular.z;
 interrupts();
 }

//set message types
std_msgs::Int16MultiArray wheel_ticks;
std_msgs::Float32MultiArray vel_trans;
sensor_msgs::Range sonar_dist;
sensor_msgs::Imu imu_msg;
sensor_msgs::Temperature temp_msg;

//initialize subscribers and publishers
ros::Subscriber<geometry_msgs::Twist> sub("twist", &messageCb);

ros::Publisher pub_ticks("ticks", &wheel_ticks);
ros::Publisher pub_range_back("sonar_back", &sonar_dist);
ros::Publisher pub_range_front("sonar_front", &sonar_dist);
ros::Publisher pub_range_left("sonar_left", &sonar_dist);
ros::Publisher pub_range_right("sonar_right", &sonar_dist);
ros::Publisher pub_vel("v_filtered", &vel_trans);
ros::Publisher imu_data("/imu_data", &imu_msg);
ros::Publisher temp_data("/temp", &temp_msg);

void setup(){
  startAFMS();
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  while(!nh.connected()) {nh.spinOnce();}
  nh.subscribe(sub);
  nh.advertise(pub_ticks);
  nh.advertise(pub_vel);
  nh.advertise(pub_range_back);
  nh.advertise(pub_range_front);
  nh.advertise(pub_range_left);
  nh.advertise(pub_range_right);
  nh.advertise(imu_data);
  nh.advertise(temp_data);
  nh.negotiateTopics();

  //range stuff
  sonar_dist.radiation_type = sensor_msgs::Range::ULTRASOUND;
  sonar_dist.min_range = 0.02;
  sonar_dist.max_range = 4.0;
  char frame_id[] = "/ultrasound_ranger";
  sonar_dist.header.frame_id = frame_id;
  sonar_dist.field_of_view = 0.523599;

  //imu calibration
  calib.accelBias[0] = -0.05;
  calib.accelBias[1] = -0.03;
  calib.accelBias[2] = 0.16;
  calib.gyroBias[0]= -8.24;
  calib.gyroBias[1]= 3.26;
  calib.gyroBias[2] = -3.68;
  calib.valid = 1;
  int err = IMU.init(calib, IMU_ADDRESS);
  err = IMU.setGyroRange(500);      //USE THESE TO SET THE RANGE, IF AN INVALID RANGE IS SET IT WILL RETURN -1
  err = IMU.setAccelRange(2);
  char frame_imu[] = "/imu_frame";
  imu_msg.header.frame_id = frame_imu;
  //change these to the pid for vx vy wz
  pid_vx.SetMode(AUTOMATIC);
  pid_vx.SetOutputLimits(0,255);
  pid_vx.SetSampleTime(10);
  pid_vy.SetMode(AUTOMATIC);
  pid_vy.SetOutputLimits(0,255);
  pid_vy.SetSampleTime(10);
  pid_wz.SetMode(AUTOMATIC);
  pid_wz.SetOutputLimits(0,255);
  pid_wz.SetSampleTime(10);

  for (int k = 0; k < 4; k++){
    //set all motors to 0
    set_vel(0);
    pinMode(encA[k], INPUT);
    pinMode(encB[k], INPUT);}

    attachInterrupt(digitalPinToInterrupt(encA[0]),readEncoder<0>,RISING);
    attachInterrupt(digitalPinToInterrupt(encA[1]),readEncoder<1>,RISING);
    attachInterrupt(digitalPinToInterrupt(encA[2]),readEncoder<2>,RISING);
    attachInterrupt(digitalPinToInterrupt(encA[3]),readEncoder<3>,RISING);
  }


void loop(){

  //read position
  int pos[4];
  noInterrupts();
  for(int k = 0; k < 4; k++){
     pos[k] = newPosition[k];}

  wheel_ticks.data = newPosition;
  interrupts();
  wheel_ticks.data_length = 4;
  // time
  long t1 = micros();
  v_plan_x = 100* math.sin(t1);
  v_plan_y = 0.0;
  w_plan = 0.0;
  float deltaT = ((float) (t1-t0))/(1.0e6);
  //evaluate pids
  pid_vx.SetControllerDirection(DIRECT);
  if(v_plan_x - v_backx < 0){
    pid_vx.SetControllerDirection(REVERSE);}
  pid_vx.Compute();

  pid_vy.SetControllerDirection(DIRECT);
  if(v_plan_y -v_backy < 0){
    pid_vy.SetControllerDirection(REVERSE);}
  pid_vy.Compute();

  pid_wz.SetControllerDirection(DIRECT);
  if(w_plan - wz_rad < 0){
    pid_wz.SetControllerDirection(REVERSE);}
  pid_wz.Compute();

  // inverse kinematics
   pwr[1] = (1/R) * (vx -vy -(sum_d *wz));
   pwr[0] = (1/R) * (vx +vy +(sum_d *wz));
   pwr[2] = (1/R) * (vx +vy -(sum_d *wz));
   pwr[3] = (1/R) * (vx -vy +(sum_d *wz));
   // set pids

  // loop through the motors
  for (int k = 0; k < 4; k++){
    //compute the speed
    velocity[k] = (pos[k]-posPrev[k])/deltaT;
    posPrev[k] = pos[k];
    // compute distance in m
    distance[k] = pos[k]/330 * 0.251;
    t0 = t1;

    //get speed in m/s
    v_cm[k] = velocity[k]/330 * 0.251;

    v_rpm[k] = velocity[k]/330 * 60.0;

    vFilt[k] = f.filterIn(v_rpm[k]);
    vPrev[k] = vFilt[k];
    //loop through the motors
    double pwm = 0;
    if (pwr[k] < 0){
      dir = -1;
      pwm = abs(pwr[k]);
    }
    else{
      dir = 1;
      pwm = pwr[k];
    }
    setMotor(dir,pwm,k);
    vFilt_float[k] = vFilt[k];
  }

  // forard Kinematics
  v_backx = (R/4)*(vFilt[0] + vFilt[1] + vFilt[2] + vFilt[3]);
  v_backy = (R/4)*(vFilt[0] - vFilt[1] + vFilt[2] - vFilt[3]);

  //this is here because float32 multi array doesn't like doubles, but double is required for the PID function
  vel_trans.data = vFilt_float;
  vel_trans.data_length = 4;

  sonar_dist.range = read_distances(0)/ 100.0;
  sonar_dist.header.stamp = nh.now();
  pub_range_back.publish(&sonar_dist);

  sonar_dist.range = read_distances(1)/ 100.0;
  sonar_dist.header.stamp = nh.now();
  pub_range_right.publish(&sonar_dist);

  sonar_dist.range = read_distances(2)/ 100.0;
  sonar_dist.header.stamp = nh.now();
  pub_range_left.publish(&sonar_dist);

  sonar_dist.range = read_distances(3)/ 100.0;
  sonar_dist.header.stamp = nh.now();
  pub_range_front.publish(&sonar_dist);

  //get data from IMU
  IMU.update();
  IMU.getAccel(&accelData);
  imu_msg.header.stamp = nh.now();
  //convert to m/s^2
  imu_msg.linear_acceleration.x = accelData.accelY * 9.81;
  imu_msg.linear_acceleration.y = accelData.accelX * 9.81;
  imu_msg.linear_acceleration.z = accelData.accelZ * 9.81;
  IMU.getGyro(&gyroData);
  //convert to rad/s from degrees
  wz_rad = gyroData.gyroZ * 0.0174533;
  imu_msg.angular_velocity.x = gyroData.gyroY * 0.0174533;
  imu_msg.angular_velocity.y = gyroData.gyroX * 0.0174533;
  imu_msg.angular_velocity.z = gyroData.gyroZ * 0.0174533;
  temp_msg.temperature = IMU.getTemp();
  imu_data.publish(&imu_msg);
  temp_msg.header.stamp = nh.now();
  temp_data.publish(&temp_msg);
  pub_ticks.publish(&wheel_ticks);
  pub_vel.publish(&vel_trans);
  nh.spinOnce();

  delay(3);
}

template <int j>
void readEncoder(){
  int b = digitalRead(encB[j]);
  if(b > 0){
    if (newPosition[j] == encoderMax){
      newPosition[j] = encoderMin;
    }
    else{
     newPosition[j]++;
    }
  }
  else{
    if (newPosition[j] == encoderMin){
      newPosition[j] = encoderMax;
    }
    else{
      newPosition[j]--;
    }
  }
}
