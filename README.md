# Map My World Robot #

1. [Abstract](#abs)
2. [Introduction](#intro)
2. [Selecting the Robot](#sel_robot)
2. [Building the Robot](#buid_robot)
3. [Software](#soft)

[image_0]: ./images/giphy.gif
[image_1]: ./images/kinect.jpg
[image_2]: ./images/real_robot.jpg
[image_3]: ./images/car_bot_urdf.jpg

## Abstract <a id='abs'></a>

The target of this project is to create a 2D occupancy grid and 3D octomap from a provided simulated environment. Furthermore, create a simulated environment (world) to be mapped as well.

![alt text][image_0]

The RTAB-Map framework is used to allow the robot to map the environment in 3D. Also, Gazebo and ROS are used for simulation.

## Introduction <a id='intro'></a>

In robotic mapping and navigation, simultaneous localization and mapping [(SLAM)](https://en.wikipedia.org/wiki/Simultaneous_localization_and_mapping) is the computational problem of constructing or updating a map of an unknown environment while simultaneously keeping track of an agent's location within it.

This project uses [rtabmap_ros](http://wiki.ros.org/rtabmap_ros) that is a wrapper of [RTAB-Map](http://introlab.github.io/rtabmap/) to provide the mapping functionalities used by the robot to map the environment. The RTAB-Map (Real-Time Appearance-Based Mapping), is a RGB-D SLAM approach based on a global loop closure detector with real-time constraints. This package can be used to generate a 3D point clouds of the environment and/or to create a 2D occupancy grid map for navigation.

For image capture the project uses a Kinect camera (RGB-D) and, to make the Kinect publish in the scan topic, the project uses the following configuration.

![alt text][image_1]

Note that the configuration provides a method of mapping the depth point cloud into a usable scan topic that can be managed by RTAB-Map.

## Selecting the Robot <a id='sel_robot'></a>

The target of this step is to chose a robot type, build the urdf model, load it into Gazebo and run the robot simulation to map the environment using the rtabmap-ros package.
So, why not get a real robot to use as reference in the sumulation? Then we can apply the lessons learned in the real model. That sounds good. So, the figure below show the model selected.

![alt text][image_2]

It is a two differential whelled robot using two step motors. So, adding more hardware components we can use it for mapping.

## Building the Robot <a id='build_robot'></a>

The urdf model must have:

- chassis
- right wheel
- left wheel
- caster
- RGB-D camera
- RGB camera
- laser scan

Based on the list above, two files were created: car_bot.xacro and car_bot.gazebo. The car_bot.xacro contain the urdf model definition and car_bot.gazebo contain the urdf sensor definition. The Fig. 4 shows part of the car_bot.xacro. 

![alt text][image_3]

Fig. 4 - Part of robot URDF model.


