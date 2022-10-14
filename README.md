# Robot-Arduino
This repo houses the Arduino sketches which are part of my [mechanum wheel mobile robot project](https://github.com/bus0v/robot/tree/devel).
![image](https://user-images.githubusercontent.com/51008991/195841442-e2d90a69-0ccf-496e-a5b5-c91e5598295f.png)

The Arduino houses a script which is responsible for low level control. 
It controls the motors through the use of the Adafruit v2 motor shield and a PID controller. 
PID control is possible due to the quadrature encoders on the wheels. Each wheel is controlled separately.

Because of the differentiation noise that comes from using discrete encoders, I had to also implement a low pass filter on the calculated speed in order for the PID algorithm to work. 

In terms of communication the Arduino acts as a single ROS node, with multiple publishers and subscribers on it.
It first listens for a vector of 4 wheel velocities and sets it as the target for the PID algorithm. Then the robot, publishes the recorded amount of encoder ticks to the Raspberry Pi, so they can be processed by the odometry node. 

Future work:
 - collect readings from ultrasound distance sensors and send them over serial to the raspberry pi
