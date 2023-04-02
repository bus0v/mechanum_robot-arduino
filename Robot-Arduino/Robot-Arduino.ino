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
#define IMU_PUBLISH_RATE 20
#define US_PUBLISH_RATE 4
#define COMMAND_RATE 20 //hz
MPU6050 IMU;

calData calib = { 0 };  //Calibration data
AccelData accelData;    //Sensor data
GyroData gyroData;

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
unsigned long prev_cmd_rec_time = 0;
float e0 = 0;
float eInt = 0;
float vFilt_float[4];
//filter params
const float cutoff_freq   = 20000.0;  //Cutoff frequency in Hz
const float sampling_time = 0.0000001; //Sampling time in seconds.
IIR::ORDER  order  = IIR::ORDER::OD1; // Order (OD1 to OD4)

// Low-pass filter
Filter f(cutoff_freq, sampling_time, order);

double vFilt[] = {0,0,0,0};
double vPrev[] = {0,0,0,0};
double v_cm[] = {0,0,0,0};
double v_rpm[] = {0,0,0,0};
double pwr[] = {0,0,0,0};
int dir;

double kp = 1, ki = 0, kd = 0;
//5 5 0,1
//can also do 5 1 0,5??
//FR is weird
PID pids[4]= {PID(&vFilt[0],&pwr[0],&targetpid[0],kp,ki,kd,DIRECT),
PID(&vFilt[1],&pwr[1],&targetpid[1],kp,ki,kd,DIRECT),
PID(&vFilt[2],&pwr[2],&targetpid[2],kp,ki,kd,DIRECT),
PID(&vFilt[3],&pwr[3],&targetpid[3],kp,ki,kd,DIRECT)};

//instantiate the node handle
ros::NodeHandle nh;

void messageCb(const std_msgs::Float32MultiArray &speed_msg){
 noInterrupts();
 targetpid[0] = speed_msg.data[0];
 targetpid[1] = speed_msg.data[1];
 targetpid[2] = speed_msg.data[2];
 targetpid[3] = speed_msg.data[3];
 interrupts();
 prev_cmd_rec_time = millis();
 }

 void pidCb(const std_msgs::Float32MultiArray &pid_msg){
   kp = pid_msg.data[0];
   ki = pid_msg.data[1];
   kd = pid_msg.data[2];
   for (int k = 0; k < 4; k++){
     pids[k].SetTunings(kp,ki,kd);
   }
 }

//set message types
std_msgs::Int16MultiArray wheel_ticks;
std_msgs::Float32MultiArray vel_trans;
sensor_msgs::Range sonar_dist;
sensor_msgs::Imu imu_msg;
sensor_msgs::Temperature temp_msg;

//initialize subscribers and publishers
ros::Subscriber<std_msgs::Float32MultiArray> sub("motor", &messageCb);
ros::Publisher pub_ticks("ticks", &wheel_ticks);
ros::Publisher pub_range_back("sonar_back", &sonar_dist);
ros::Publisher pub_range_front("sonar_front", &sonar_dist);
ros::Publisher pub_range_left("sonar_left", &sonar_dist);
ros::Publisher pub_range_right("sonar_right", &sonar_dist);
ros::Publisher pub_vel("v_filtered", &vel_trans);
ros::Publisher imu_data("/imu/data_raw", &imu_msg);
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

  for (int k = 0; k < 4; k++){
    pids[k].SetMode(AUTOMATIC);
    pids[k].SetOutputLimits(0,255);
    pids[k].SetSampleTime(10);
    pinMode(encA[k], INPUT);
    pinMode(encB[k], INPUT);
    //kp kd ki pwr

    //2.5 0.8 0.5 works with 5% accuracy for distance
    set_vel(0);}
    attachInterrupt(digitalPinToInterrupt(encA[0]),readEncoder<0>,RISING);
    attachInterrupt(digitalPinToInterrupt(encA[1]),readEncoder<1>,RISING);
    attachInterrupt(digitalPinToInterrupt(encA[2]),readEncoder<2>,RISING);
    attachInterrupt(digitalPinToInterrupt(encA[3]),readEncoder<3>,RISING);}


void loop(){
  unsigned long prev_cmd_time = 0;
  if (millis()-prev_cmd_time >= 1000 / COMMAND_RATE){
    move();
    prev_cmd_time = millis();
  }
  if (millis()-prev_cmd_rec_time >= 1000){
    stop();
  }

  unsigned long prev_range_time = 0;
  if (millis()-prev_range_time >= 1000 / US_PUBLISH_RATE){
    publishUS();}

  unsigned long prev_imu_time = 0;
  if (millis()-prev_imu_time >= 1000 / IMU_PUBLISH_RATE){
    publishIMU();
    //add something here to check if it Initialized
  }
  //this is here because float32 multi array doesn't like doubles, but double is required for the PID function
  vel_trans.data = vFilt_float;
  vel_trans.data_length = 4;

  pub_ticks.publish(&wheel_ticks);
  pub_vel.publish(&vel_trans);
  nh.spinOnce();
  //delay(3);
}

void move(){
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
  float deltaT = ((float) (t1-t0))/(1.0e6);

  // loop through the motors
  for (int k = 0; k < 4; k++){
    //compute the speed
    velocity[k] = (pos[k]-posPrev[k])/deltaT;
    posPrev[k] = pos[k];
    t0 = t1;

    //get speed in cm/s
    v_cm[k] = velocity[k]/330*0.251;
    v_rpm[k] = velocity[k]/330*60.0;

    vFilt[k] = v_rpm[k];
    vPrev[k] = vFilt[k];

    //evaluate the control signal
    pids[k].SetControllerDirection(DIRECT);
    int dir = 1;
    if(targetpid[k] < 0){
      dir = -1;
      pids[k].SetControllerDirection(REVERSE);}
    pids[k].Compute();

    //loop through the motors
    setMotor(dir,pwr[k],k);
    // convert double to float for transmission
    vFilt_float[k] = v_cm[k];
  }
}

void stop(){
  for(int k = 0; k < 4; k++){
    vFilt[k] = 0;
  }
}

void publishUS(){
  char frame_back[] = "/back_sonar";
  sonar_dist.header.frame_id = frame_back;
  sonar_dist.range = read_distances(0)/ 100.0;
  sonar_dist.header.stamp = nh.now();
  pub_range_back.publish(&sonar_dist);

  char frame_right[] = "/right_sonar";
  sonar_dist.header.frame_id = frame_right;
  sonar_dist.range = read_distances(1)/ 100.0;
  sonar_dist.header.stamp = nh.now();
  pub_range_right.publish(&sonar_dist);

  char frame_left[] = "/left_sonar";
  sonar_dist.header.frame_id = frame_left;
  sonar_dist.range = read_distances(2)/ 100.0;
  sonar_dist.header.stamp = nh.now();
  pub_range_left.publish(&sonar_dist);

  char frame_front[] = "/front_sonar";
  sonar_dist.header.frame_id = frame_front;
  sonar_dist.range = read_distances(3)/ 100.0;
  sonar_dist.header.stamp = nh.now();
  pub_range_front.publish(&sonar_dist);
}

void publishIMU(){
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
  imu_msg.angular_velocity.x = gyroData.gyroY * 0.0174533;
  imu_msg.angular_velocity.y = gyroData.gyroX * 0.0174533;
  imu_msg.angular_velocity.z = gyroData.gyroZ * 0.0174533;
  temp_msg.temperature = IMU.getTemp();
  imu_data.publish(&imu_msg);
  temp_msg.header.stamp = nh.now();
  temp_data.publish(&temp_msg);
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
