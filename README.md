# MPC-algo-for-differential-autonomous-vehicule
The project consists of three main parts:
- the first part is given by the file MPC.cpp. it is an algorithm of the Model Predictive Control that uses Particles Swarm Optimization method to calculate the optimum robot velocity for a given reference trajectory
- the second part is within the file ref.cpp. this file contains functions that generates different trajectories (circle, straigth line)
- the third part is given by ino_cpp.cpp. this is the main code that calls out the other files and connect between the different part of the project via ROS libraries
