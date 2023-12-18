# Robot-Arduino
This repo houses the Arduino sketches which are part of my [mechanum wheel mobile robot project](https://github.com/bus0v/robot/tree/devel).
![image](https://user-images.githubusercontent.com/51008991/195841442-e2d90a69-0ccf-496e-a5b5-c91e5598295f.png)

**Robot-Arduino** folder houses the main script while the others are used for testing
In terms of communication the Arduino acts as a single ROS node, with multiple publishers and subscribers on it.

It controls the motors through the use of the Adafruit v2 motor shield and a PID controller. 
PID control is possible due to the quadrature encoders on the wheels. Each wheel is controlled separately.

The arduino recieves an array of 4 floats from the rasberry pi on the **/motor** topic, which are the desired motor speed. 
Using a specified command rate the speed is published to the motors and then controlled through PID using the encoder feedback. 
Then the robot, publishes the recorded amount of encoder ticks to the Raspberry Pi to the **/ticks** topic, so they can be processed by the odometry node. 

The Arduino also collects IMU information and publishes it on the **/imu_data** topic. 

It also has 4 ultra sound sensors that I added for redundance that publish to their own topics, but they are pretty unreliable.


 
