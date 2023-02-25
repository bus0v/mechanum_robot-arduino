#include <Arduino.h>
#include <Adafruit_MotorShield.h>
// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *FR = AFMS.getMotor(1);
Adafruit_DCMotor *BL = AFMS.getMotor(2);
Adafruit_DCMotor *BR = AFMS.getMotor(3);
Adafruit_DCMotor *FL = AFMS.getMotor(4);
Adafruit_DCMotor motors[] = {*FR,*FL,*BL,*BR};

void startAFMS(){
  AFMS.begin();
  if (!AFMS.begin()) {         // create with the default frequency 1.6KHz
  // if (!AFMS.begin(1000)) {  // OR with a different frequency, say 1KHz
    while (1);
  }
  
}

void setMotor(int dir,double pwm,int k){
  int pwmVal;
  float trim_left = 1;
  pwmVal = pwm;
  if ((k==1) || (k==2)){
    pwmVal = pwmVal * trim_left;
  }
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
void set_vel(int vel){
  for (int k = 0; k < 4; k++){
    motors[k].setSpeed(vel);
  }
}
