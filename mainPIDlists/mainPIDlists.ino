 #include <ros.h>
#include <std_msgs/UInt16.h>
//FR,FL,BL,BR

// define pin lists
const int encA[] = {18, 21, 3, 20};
const int encB[] = {19, 22, 2, 23};
const int pwm[] = {5, 44, 6, 46};
const int in1[] = {7, 35, 10, 33};
const int in2[] = {8, 37, 11, 31};

volatile int newPosition [] = {0,0,0,0};

long t0 = 0;
float e0 = 0;
float eInt = 0;

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
    int e = target - value;
    
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

void setup(){ 
  Serial.begin(9600);
  for (int k = 0; k < 4; k++){
    pinMode(encA[k], INPUT);
    pinMode(encB[k], INPUT);
    pinMode(pwm[k], OUTPUT);
    pinMode(in1[k], OUTPUT);
    pinMode(in2[k], OUTPUT);
    pid[k].setParams(2.5,0.8,0.5,255);
    }
    //2.5 0.8 0.5 works with 5% accuracy
    attachInterrupt(digitalPinToInterrupt(encA[0]),readEncoder<0>,RISING);
    attachInterrupt(digitalPinToInterrupt(encA[1]),readEncoder<1>,RISING);
    attachInterrupt(digitalPinToInterrupt(encA[2]),readEncoder<2>,RISING);
    attachInterrupt(digitalPinToInterrupt(encA[3]),readEncoder<3>,RISING);
    
  
}



void loop(){  

  //setMotor(dir, pwr, LenB, blIN1, blIN2);
  int target[4];
  //750*sin(t0/1e6);
  target[0] = -330*1.05*5;
  target[1] = 330*5;
  target[2] = -330*5;
  target[3] = 330*1.05*5;

  // time
  long t1 = micros();
  float deltaT = ((float) (t1-t0))/(1.0e6);
  t0 = t1;
  
  
  //read position
  int pos[4];
  noInterrupts();
  for(int k = 0; k < 4; k++){
     pos[k] = newPosition[k];
    }
  interrupts();
  
  // loop through the motors
  
  for (int k=0; k<4; k++){
    int pwr,dir;
    //evaluate the control signal
    pid[k].evalu(pos[k],target[k],deltaT,pwr,dir);
    setMotor(dir,pwr,pwm[k],in1[k],in2[k]);
  }
  // make this into a loop
  Serial.print(" FR    FL    BL    BR ");
  Serial.println();
  for (int p = 0; p < 4; p++){
    Serial.print(pos[p]);
    Serial.print(" ");
    Serial.print(target[p]);
    Serial.print(" ");
    Serial.print(newPosition[p]);
    Serial.print(" ");
  } 
  Serial.println();
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
  
