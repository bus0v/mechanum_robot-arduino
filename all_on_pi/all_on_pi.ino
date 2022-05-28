#include <ros.h>
#include <ros/time.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int32MultiArray.h>
//FR,FL,BL,BR
//May 26

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


;
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
const std_msgs::Int32MultiArray ticks_array;
ticks_array.layout.data_offset = 0
ticks_array.layout.dim = [MultiArrayDimension()]
ticks_array.layout.dim[0].label = "encoder"
ticks_array.layout.dim[0].size = 4
ros::Pulisher<std_msgs::Int32MultiArray> pub("ticks", &ticks_array)

void setup(){
  Serial.begin(57600);

  for (int k = 0; k < 4; k++){
    pinMode(encA[k], INPUT);
    pinMode(encB[k], INPUT);
    pinMode(pwm[k], OUTPUT);
    pinMode(in1[k], OUTPUT);
    pinMode(in2[k], OUTPUT);

    attachInterrupt(digitalPinToInterrupt(encA[0]),readEncoder<0>,RISING);
    attachInterrupt(digitalPinToInterrupt(encA[1]),readEncoder<1>,RISING);
    attachInterrupt(digitalPinToInterrupt(encA[2]),readEncoder<2>,RISING);
    attachInterrupt(digitalPinToInterrupt(encA[3]),readEncoder<3>,RISING);

    nh.initNode();
    nh.subscribe(sub);
    nh.advertise(pub)
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
  ticks_array.data = pos;


  for (int k=0; k<4; k++){
    //evaluate the control signal
    int dir = 1;
    if target k <0:
      dir = -1;
  setMotor(dir,target[k],pwm[k],in1[k],in2[k]);
  }
  pub.publish( &ticks_array)
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
