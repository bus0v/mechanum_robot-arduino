#include <ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <filters.h>
#include <PID_v1.h>
#include <std_msgs/Int16MultiArray.h>
#include "motors.h"
#include "ultrasound_sensors.h"

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

double kp = 20, ki = 5, kd = 0.5;
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
 }

std_msgs::Int16MultiArray wheel_ticks;
//std_msgs::Float32MultiArray vel_trans;
//std_msgs::Float32MultiArray sonar_dist;
ros::Subscriber<std_msgs::Float32MultiArray> sub("motor", &messageCb);
ros::Publisher pub_ticks("ticks", &wheel_ticks);
//ros::Publisher pub_range("sonar_readings", &sonar_dist);
//ros::Publisher pub_vel("v_filtered", &vel_trans);

void setup(){
  startAFMS();
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  while(!nh.connected()) {nh.spinOnce();}
  nh.subscribe(sub);
  nh.advertise(pub_ticks);
  nh.negotiateTopics();
  //nh.advertise(pub_vel);
  //nh.advertise(pub_range); 
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
    // compute distance in cm
    distance[k] = pos[k]/330 * 25.1;
    t0 = t1;

    //get speed in cm/s
    v_cm[k] = velocity[k]/330*25.1;

    v_rpm[k] = velocity[k]/330*60.0;

    vFilt[k] = f.filterIn(v_rpm[k]);
    vPrev[k] = vFilt[k];
    //vel_trans.data[k] = vFilt[k];
    //evaluate the control signal
    pids[k].SetControllerDirection(DIRECT);
    int dir = 1;
    if(targetpid[k]<0){
      dir = -1;
      pids[k].SetControllerDirection(REVERSE);}
    pids[k].Compute();

    //loop through the motors
    setMotor(dir,pwr[k],k);
  }
  //sonar_dist.data = *read_distances();
  noInterrupts();
  nh.spinOnce();
  pub_ticks.publish(&wheel_ticks);
  interrupts();
  delay(3);
  //pub_vel.publish(&vel_trans);
  //pub_range.publish(&sonar_dist);
  
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
