# MPC-algo-for-differential-autonomous-vehicule
The project consists of four main parts:
- the first part is given by the file MPC.cpp, it is an algorithm of the Model Predictive Control that uses Particles Swarm Optimization method to calculate the optimum robot velocity for a given reference trajectory
- the second part is within the file ref.cpp, this file contains functions that generates different trajectories (circle, straigth line)
- the third part is given by main.cpp, this is the main code that calls out the other files and connect between the different part of the project via ROS libraries
- the fourth and last part is the arduino code. It a .ino file that acquires the state of the robot (position and orientation) from the sensors (encoders + gyroscope) and estimates the odometry of the robot. It also implement an inner loop PID controller that regulates the velocity of the motors, such that the desired velocity will be communicated from the MPC command (Raspberry node)  via ROS


PS: the folder ino_cpp is a ros package. the executable "main" produced by this package should be executed along with ros master and either arduino or Gazebo simulation 
