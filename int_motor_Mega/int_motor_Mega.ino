#include <ros.h>
#include <std_msgs/UInt16.h>
#include <Encoder.h>

// define pin names
#define enB 46
#define enA 44
#define LenA 5
#define LenB 6
#define FRB 18
#define FRA 19
#define BRB 20
#define BRA 23
#define FLA 21
#define FLB 22
#define BLB 2
#define BLA 3

#define brIN1 31
#define brIN2 33
#define flIN1 35
#define flIN2 37
#define frIN1 7
#define frIN2 8
#define blIN1 9
#define blIN2 10

Encoder FLEnc(21, 22);
Encoder FREnc(18, 19);
Encoder BREnc(20, 23);
Encoder BLEnc(3, 2);

// instantiate the node handle
ros::NodeHandle nh;

void messageCb( const std_msgs::UInt16& speed_msg){
  int pwr = speed_msg.data;
  digitalWrite(13, HIGH-digitalRead(13));
  analogWrite(enA, pwr);
  fl_forward();   // move forward 0.5 second
  delay(500);
  fl_stop();
}

ros::Subscriber<std_msgs::UInt16> sub("motor", &messageCb );

void setup()
{ 
  Serial.begin(57600);
  pinMode(enA, OUTPUT);
  pinMode(LenA, OUTPUT);
  pinMode(LenB, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(brIN1, OUTPUT);
  pinMode(brIN2, OUTPUT);  
  pinMode(flIN1, OUTPUT);
  pinMode(flIN2, OUTPUT);
  pinMode(frIN1, OUTPUT);
  pinMode(frIN2, OUTPUT);
  pinMode(blIN1, OUTPUT);
  pinMode(blIN2, OUTPUT);

  int power = 110;
  analogWrite(enA, power);
  analogWrite(enB, power);
  analogWrite(LenA, power);
  analogWrite(LenB, power);

  fl_stop();
  fr_stop();
  br_stop();
  bl_stop();
  nh.initNode();
  nh.subscribe(sub);
}

long oldPosition  = -999;

void loop()
{  
  long newPosition = BLEnc.read();
  if (newPosition != oldPosition) {
    oldPosition = newPosition;
    Serial.println(newPosition);
  }
  if (newPosition < 300){
    forward();  
  }
  else{
    stop_all();
  }
  
}

void test(){
  //test each wheel forward and backward
  fl_forward();
  delay(500);
  fl_back();
  delay(500);
  stop_all();
  bl_forward();
  delay(500);
  bl_back();
  delay(500);
  stop_all();
  fr_forward();
  delay(500);
  fr_back();
  delay(500);
  stop_all();
  br_forward();
  delay(500);
  br_back();
  delay(600);
  stop_all();
  
}
void backward(int wait){      
      fl_back();
      fr_back();
      bl_back();
      br_back();
      delay(wait);
} 

void stop_all(){
      fl_stop();
      fr_stop();
      bl_stop();
      br_stop();
}
void forward(){
      fl_forward();
      fr_forward();
      bl_forward();
      br_forward();
      
}
void turn_left(int wait){
  fl_forward();
  bl_forward();
  fr_back();
  br_back();
  delay(wait);
    
}
void turn_right(int wait){
  fr_forward();
  br_forward();
  fl_back();
  bl_back();
  delay(wait);
}
void slide_right(int wait){
  fl_forward();
  br_forward();
  fr_back();
  bl_back();
  delay(wait);
}
void slide_left(int wait){
  fr_forward();
  bl_forward();
  fl_back();
  br_back();
  delay(wait);
}
void diag_right_f(int wait){
  fl_forward();
  br_forward();
  delay(wait);
}
void diag_right_b(int wait){
  fl_back();
  br_back();
  delay(wait);
}
void diag_left_f(int wait){
  fr_forward();
  bl_forward();
  delay(wait);
}
void diag_left_b(int wait){
  fr_back();
  bl_back();
  delay(wait);
}
void fl_forward(){
      digitalWrite(flIN1, HIGH);
      digitalWrite(flIN2, LOW);
}
void fl_stop(){
      digitalWrite(flIN1, LOW);
      digitalWrite(flIN2, LOW);
}
void fl_back(){
      digitalWrite(flIN1, LOW);
      digitalWrite(flIN2, HIGH);
}
void br_back(){
     //works
      digitalWrite(brIN1, HIGH);
      digitalWrite(brIN2, LOW);
}
void br_forward(){ 
      digitalWrite(brIN1, LOW);
      digitalWrite(brIN2, HIGH);
}
void br_stop(){
      digitalWrite(brIN2, LOW);
      digitalWrite(brIN1, LOW);  
}
void fr_back(){
      digitalWrite(frIN1, LOW);
      digitalWrite(frIN2, HIGH);
}
void fr_forward(){
      digitalWrite(frIN1, HIGH);
      digitalWrite(frIN2, LOW);
}
void fr_stop(){
      digitalWrite(frIN1, LOW);
      digitalWrite(frIN2, LOW);
}
void bl_back(){
      digitalWrite(blIN1, LOW);
      digitalWrite(blIN2, HIGH);
}
void bl_forward(){
      digitalWrite(blIN1, HIGH);
      digitalWrite(blIN2, LOW);
}
void bl_stop(){
      digitalWrite(blIN1, LOW);
      digitalWrite(blIN2, LOW);
}
