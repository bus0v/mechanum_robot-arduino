
#include <Adafruit_MotorShield.h>
#include <ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <filters.h>

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield();


//FR,FL,BL,BR
// define pin lists
const int encA[] = {3, 19, 18, 2};
const int encB[] = {26, 15, 10, 5};
int target[4] = {0,0,0,0};
volatile int newPosition [] = {0,0,0,0};
volatile int velocity[] = {0,0,0,0};
volatile float distance[] = {0,0,0,0};
volatile int posPrev[] = {0,0,0,0};
long t0 = 0;
float e0 = 0;
float eInt = 0;

Adafruit_DCMotor *FR = AFMS.getMotor(1);
Adafruit_DCMotor *BL = AFMS.getMotor(2);
Adafruit_DCMotor *BR = AFMS.getMotor(3);
Adafruit_DCMotor *FL = AFMS.getMotor(4);

Adafruit_DCMotor motors[]={*FR,*BL,*BR,*FL};


//filter params
const float cutoff_freq   = 20.0;  //Cutoff frequency in Hz
const float sampling_time = 0.00005; //Sampling time in seconds.
IIR::ORDER  order  = IIR::ORDER::OD3; // Order (OD1 to OD4)

// Low-pass filter
Filter f(cutoff_freq, sampling_time, order);

float vFilt[] = {0,0,0,0};
float vPrev[] = {0,0,0,0};
float v_cm[] = {0,0,0,0};
float v_rpm[] = {0,0,0,0};

class SimplePID{
  private:
    float Kp, Kd, Ki, umax;
    float e0, eInt;
  public:
  //Constructor
  SimplePID(): Kp(5),Kd(5),Ki(0),umax(255),e0(0.0),eInt(0.0){}

  //This function sets parameters
  void setParams(float kpIn, float kdIn, float kiIn, float umaxIn){
    Kp = kpIn; Kd = kdIn; Ki = kiIn; umax = umaxIn;
  }

  void evalu(int value, int target, float deltaT, int &pwr, int &dir){
    //errors
    float e = target - value;
    eInt = eInt + e * deltaT;
    float dedt = (e-e0)/deltaT;
    float u = Kp*e + Kd*dedt + Ki*eInt;
    //motor power
    pwr = fabs(u);
    if (pwr > umax){
      pwr = umax;
    }
    dir = 1;
    if (u<0){
      dir = -1;
    }
    float targ = target;
    if (fabs(e/targ)<0.01){
      dir = 0;
    }
    e0 = e;
}};

//instantiate class
SimplePID pid[4];
<<<<<<< HEAD


//instantiate the node handle
ros::NodeHandle nh;
=======
long t0 = 0;
float e0 = 0;
float eInt = 0;

//instantiate the node handle
ros::NodeHandle nh;

void messageCb(const std_msgs::Float32MultiArray &speed_msg){
 target[0] = speed_msg.data[0];
 target[1] = speed_msg.data[1];
 target[2] = speed_msg.data[2];
 target[3] = speed_msg.data[3];
}
>>>>>>> main

void messageCb(const std_msgs::Float32MultiArray &speed_msg){
 target[0] = speed_msg.data[0];
 target[1] = speed_msg.data[1];
 target[2] = speed_msg.data[2];
 target[3] = speed_msg.data[3];
}
ros::Subscriber<std_msgs::Float32MultiArray> sub("motor", &messageCb );
void setup(){
<<<<<<< HEAD
  Serial.begin(57600);
=======
  Serial.begin(230400);
>>>>>>> main
  AFMS.begin();

  if (!AFMS.begin()) {         // create with the default frequency 1.6KHz
  // if (!AFMS.begin(1000)) {  // OR with a different frequency, say 1KHz
    Serial.println("Could not find Motor Shield. Check wiring.");
    while (1);
  }
  Serial.println("Motor Shield found.");

  for (int k = 0; k < 4; k++){
    pinMode(encA[k], INPUT);
    pinMode(encB[k], INPUT);
    pid[k].setParams(1.5, 0.5, 0, 255);
    //2.5 0.8 0.5 works with 5% accuracy for distance
    attachInterrupt(digitalPinToInterrupt(encA[0]),readEncoder<0>,RISING);
    attachInterrupt(digitalPinToInterrupt(encA[1]),readEncoder<1>,RISING);
    attachInterrupt(digitalPinToInterrupt(encA[2]),readEncoder<2>,RISING);
    attachInterrupt(digitalPinToInterrupt(encA[3]),readEncoder<3>,RISING);
    motors[k].setSpeed(0);
    nh.initNode();
    nh.subscribe(sub);
    while(!nh.connected()) {nh.spinOnce();}

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

    // loop through the motors
      int pwr,dir;
      //evaluate the control signal
      pid[k].evalu(vFilt[k],target[k],deltaT,pwr,dir);
    setMotor(dir,pwr,k);
  }

  // //print output
  // Serial.print("FR   FL    BL    BR ");
  // Serial.println();
  // for (int p = 0; p < 4; p++){
  //   Serial.print(newPosition[p]);
  //   Serial.print(" ");
  // }
  // Serial.println();
  nh.spinOnce();
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

void setMotor(int dir,int pwmVal,int k){
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
