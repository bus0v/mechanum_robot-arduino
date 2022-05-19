#include <ros.h>
#include <std_msgs/Float32MultiArray.h>
//FR,FL,BL,BR
//April 1st

// define pin lists
const int encA[] = {18, 21, 3, 20};
const int encB[] = {19, 22, 2, 23};
const int pwm[] = {5, 44, 6, 46};
const int in1[] = {7, 35, 10, 33};
const int in2[] = {8, 37, 11, 31};
int target[4] = {0,0,0,0};
volatile int newPosition [] = {0,0,0,0};
volatile int velocity[] = {0,0,0,0};
float distance[] = {0,0,0,0};
int posPrev[] = {0,0,0,0};
volatile long t0 = 0;
float e0 = 0;
float eInt = 0;

float vFilt[] = {0,0,0,0};
float vPrev[] = {0,0,0,0};
float vcm[] = {0,0,0,0};
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
// instantiate the node handle
ros::NodeHandle nh;

void messageCb(const std_msgs::Float32MultiArray &speed_msg){
  target[0] = speed_msg.data[0];
  target[1] = speed_msg.data[1];
  target[2] = speed_msg.data[2];
  target[3] = speed_msg.data[3];
}
//topic name = motor
ros::Subscriber<std_msgs::Float32MultiArray> sub("motor", &messageCb );

void setup(){
  Serial.begin(57600);

  for (int k = 0; k < 4; k++){
    pinMode(encA[k], INPUT);
    pinMode(encB[k], INPUT);
    pinMode(pwm[k], OUTPUT);
    pinMode(in1[k], OUTPUT);
    pinMode(in2[k], OUTPUT);
    //kp   kd   ki
    pid[k].setParams(10,0.5,10,255);
    }
    //10 0.5 10 works for speed
    //2.5 0.8 0.5 works with 5% accuracy for distance
    attachInterrupt(digitalPinToInterrupt(encA[0]),readEncoder<0>,RISING);
    attachInterrupt(digitalPinToInterrupt(encA[1]),readEncoder<1>,RISING);
    attachInterrupt(digitalPinToInterrupt(encA[2]),readEncoder<2>,RISING);
    attachInterrupt(digitalPinToInterrupt(encA[3]),readEncoder<3>,RISING);

    nh.initNode();
    nh.subscribe(sub);
    while(!nh.connected()) {nh.spinOnce();}
}

void loop(){

  //read position
  int pos[4] = {0,0,0,0};
  noInterrupts();
  for(int k = 0; k < 4; k++){
     pos[k] = newPosition[k];
    }
  interrupts();


for (int k=0; k<4; k++){
  // time
  long t1 = micros();
  float deltaT = ((float) (t1-t0))/(1.0e6);

  //compute the speed
  velocity[k] = (pos[k]-posPrev[k])/deltaT;
  posPrev[k] = pos[k];
  // compute distance in cm
  distance[k] = pos[k]/330 * 25.1;
  t0 = t1;

  //get speed in cm/s
  vcm[k] = velocity[k]/330.0*25.1;
  // Low-pass filter (25 Hz cutoff)
  vFilt[k] = 0.854*vFilt[k] + 0.0728*vcm[k] + 0.0728*vPrev[k];
  vPrev[k] = vFilt[k];

  // loop through the motors
  int pwr,dir;
  //evaluate the control signal
  pid[k].evalu(vFilt[k],target[k],deltaT,pwr,dir);
  setMotor(dir,pwr,pwm[k],in1[k],in2[k]);
  }


  Serial.print(" vFilt    target    vcm ");
  Serial.println();
  for (int p = 0; p < 4; p++){
    Serial.print(vFilt[p]);
    Serial.print(" ");
    Serial.print(target[p]);
    Serial.print(" ");
    Serial.print(vcm[p]);
  }
  Serial.println();
  nh.spinOnce();
  delay(1);

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

void setMotor(int dir,int pwmVal,int pwm, int IN1,int IN2){
  analogWrite(pwm, pwmVal);
  if (dir == 1){
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, LOW);
  }
  else if (dir == -1){
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, HIGH);
  }
  else{
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, LOW);
  }
}
