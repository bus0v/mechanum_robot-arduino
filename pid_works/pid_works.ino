
#include <Adafruit_MotorShield.h>
#include <ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <filters.h>
#include <PID_v1.h>


// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield();


//FR,FL,BL,BR
// define pin lists
const int encA[] = {3, 19, 18, 2};
const int encB[] = {26, 15, 10, 5};
double target[4] = {0,0,0,0};
volatile int newPosition [] = {0,0,0,0};
volatile float velocity[] = {0,0,0,0};
volatile float distance[] = {0,0,0,0};
volatile int posPrev[] = {0,0,0,0};
long t0 = 0;
float e0 = 0;
float eInt = 0;

Adafruit_DCMotor *FR = AFMS.getMotor(1);
Adafruit_DCMotor *BL = AFMS.getMotor(2);
Adafruit_DCMotor *BR = AFMS.getMotor(3);
Adafruit_DCMotor *FL = AFMS.getMotor(4);

Adafruit_DCMotor motors[] = {*FR,*BL,*BR,*FL};


//filter params
const float cutoff_freq   = 15000.0;  //Cutoff frequency in Hz
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
double kp = 5,ki=1,kd=0.5;

PID pids[4]= {PID(&vFilt[0],&pwr[0],&target[0],kp,ki,kd,DIRECT),
PID(&vFilt[1],&pwr[1],&target[1],kp,ki,kd,DIRECT),
PID(&vFilt[2],&pwr[2],&target[2],kp,ki,kd,DIRECT),
PID(&vFilt[3],&pwr[3],&target[3],kp,ki,kd,DIRECT)};

//instantiate the node handle
//ros::NodeHandle nh;


void messageCb(const std_msgs::Float32MultiArray &speed_msg){
 target[0] = speed_msg.data[0];
 target[1] = speed_msg.data[1];
 target[2] = speed_msg.data[2];
 target[3] = speed_msg.data[3];
}
//ros::Subscriber<std_msgs::Float32MultiArray> sub("motor", &messageCb );

void setup(){

  Serial.begin(57600);

  AFMS.begin();

  if (!AFMS.begin()) {         // create with the default frequency 1.6KHz
  // if (!AFMS.begin(1000)) {  // OR with a different frequency, say 1KHz
    Serial.println("Could not find Motor Shield. Check wiring.");
    while (1);
  }
  //Serial.println("Motor Shield found.");

  for (int k = 0; k < 4; k++){
    pids[k].SetMode(AUTOMATIC);
    pids[k].SetOutputLimits(-255,255);
    pids[k].SetSampleTime(10);
    pinMode(encA[k], INPUT);
    pinMode(encB[k], INPUT);
    //kp kd ki pwr

    //2.5 0.8 0.5 works with 5% accuracy for distance
    attachInterrupt(digitalPinToInterrupt(encA[0]),readEncoder<0>,RISING);
    attachInterrupt(digitalPinToInterrupt(encA[1]),readEncoder<1>,RISING);
    attachInterrupt(digitalPinToInterrupt(encA[2]),readEncoder<2>,RISING);
    attachInterrupt(digitalPinToInterrupt(encA[3]),readEncoder<3>,RISING);
    motors[k].setSpeed(0);
//    nh.initNode();
//    nh.subscribe(sub);
//    while(!nh.connected()) {nh.spinOnce();}

}}


void loop(){
  //read position
  int pos[4];
  noInterrupts();
  for(int k = 0; k < 4; k++){
     pos[k] = newPosition[k];
    }
  interrupts();

  // time
  long t1 = micros();
  float deltaT = ((float) (t1-t0))/(1.0e6);


  target[0] = 30*(sin(t1/2e6));
  target[1] = 30*(sin(t1/2e6));
  target[2] = 30*(sin(t1/2e6));
  target[3] = 30*(sin(t1/2e6));

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
    

    
    //evaluate the control signal
    pids[k].SetControllerDirection(DIRECT);
    int dir = 1;
    if(target[k]<0){
      dir = -1;
      pids[k].SetControllerDirection(REVERSE);}
    pids[k].Compute();

    //old pid[k].evalu(vFilt[k],target[k],deltaT,pwr,dir);
     //loop through the motors
    setMotor(dir,pwr[k],k);
  }
  //FR only
  Serial.print("Target v_rpm V_filt");
  Serial.println();
  Serial.print(target[0]);
  Serial.print(" ");
  Serial.print(vFilt[0]);
  Serial.println();
//print output
//   Serial.print("Target FR   FL    BL    BR ");
//   Serial.println();
//   Serial.print(target[0]);
//   Serial.print(" ");
//
//   for (int p = 0; p < 4; p++){
//    Serial.print(vFilt[p]);
//    Serial.print(" ");
//   }
//   Serial.println();
//  nh.spinOnce();
//  delay(3);
}

template <int j>
void readEncoder(){
  int b = digitalRead(encB[j]);
  if(b > 0){
    newPosition[j]++;
  }
  else{
    newPosition[j]--;
  }
}

void setMotor(int dir,double pwm,int k){
  int pwmVal;
  pwmVal=pwm;
  motors[k].setSpeed(pwmVal);
  if (dir == 1){
    motors[k].run(FORWARD);
  }
  else if (dir == -1){
    motors[k].run(BACKWARD);
  }
  else{
    motors[k].run(RELEASE);
  }
}
