# Map My World Robot #

1. [Abstract](#abs)
2. [Background](#back)
3. [Introduction](#intro)
4. [Selecting the Robot](#sel_robot)
5. [Building the Robot](#buid_robot)
6. [World: the provided world](#world)
7. [World: creating a new world](#new_world)
8. [Creating the launch files](#launch)
9. [Robot, map the world](#map)
10. [Results](#result)


[image_0]: ./images/giphy.gif
[image_1]: ./images/kinect.jpg
[image_2]: ./images/real_robot.jpg
[image_3]: ./images/car_bot_urdf.jpg
[image_4]: ./images/frames.jpg
[image_5]: ./images/robot_view.jpg
[image_6]: ./images/kitchen_dinig_world.jpg
[image_7]: ./images/created_world.jpg
[image_8]: ./images/directories.jpg
[image_9]: ./images/all.jpg


## Abstract <a id='abs'></a>

The target of this project is to create a 2D occupancy grid and 3D octomap from a provided simulated environment. Furthermore, create a simulated environment (world) to be mapped as well.

![alt text][image_0]

The RTAB-Map framework is used to allow the robot to map the environment in 3D. Also, Gazebo and ROS are used for simulation.

## Background <a id='back'></a>

In robotic mapping and navigation, the simultaneous localization and mapping [(SLAM)](https://en.wikipedia.org/wiki/Simultaneous_localization_and_mapping) is the computational problem of constructing or updating a map of an unknown environment while simultaneously keeping track of an agent's location within it. We need a map to localize and we need to localize to make the map.

## Introduction <a id='intro'></a>



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

To see the final urdf model we can take a look the picture of frames in Fig. 5 that shows the robot model has a camera and a rgbd frames.

![alt text][image_4]

Fig. 5 - Robot frames.

Using xacro elements and macros in xacro file, we can contruct the entire urdf model and load it into Gazebo tool. The result is in Fig. 6.

![alt text][image_5]

Fig. 6 - The robot model spawned in Gazebo tool.

## World: the provided world <a id='world'></a>

As the project title states, the project target is to map the world. So we have two worlds to be managed: the provided world and a new one that must be createded. The Fig. 7 shows the provided kitchen dining  world.

![alt text][image_6]

Fig. 7 - The kitchen dining world

Note that the environment has a lot of features which will demand a heavy hardware resource in the mapping process.

## World: Creating a new world <a id='new_world'></a>

To create a new world we can use the `Build Editor` option in Gazebo tool. In the editor we can use existing objects and models or create new objects using the features provided in the editor tool. So, using walls, tables and others objects, we created a new world as showed in Fig. 8.

![alt text][image_7]

Fig. 8 - The new created world

## Creating the launch files <a id='launch'></a>

At this point we have the robot model and the world environments. To manage the mapping process we need load the robot model, the world, launch the Gazebo simulation, launch the rtabmap-ros package and a teleop script to teleoperate the robot in the environment to allow it gather as much features, from the world, as possible.

To make the things easier, we can automate these steps creating launch files that will contain a set of commands that are executed using the `roslaunch` tool.

For illustration, the following snippet code is the world.launch content.

```XML
<?xml version="1.0"?>
<launch>
  <arg name="world" default="empty"/> 
  <arg name="paused" default="false"/>
  <arg name="use_sim_time" default="true"/>
  <arg name="gui" default="true"/>
  <arg name="headless" default="false"/>
  <arg name="debug" default="false"/>

<!-- We resume the logic in empty_world.launch, changing only the name of the world to be launched -->
  <include file="$(find gazebo_ros)/launch/empty_world.launch">
    <!-- <arg name="world_name" value="$(find slam_project)/worlds/kitchen_dining.world"/> -->
    <arg name="world_name" value="$(find slam_project)/worlds/user_created.world"/>
    <arg name="paused" value="$(arg paused)"/>
    <arg name="use_sim_time" value="$(arg use_sim_time)"/>
    <arg name="gui" value="$(arg gui)"/>
    <arg name="headless" value="$(arg headless)"/>
    <arg name="debug" value="$(arg debug)"/>
  </include>

  <!-- spawn a robot in gazebo world -->
  <include file="$(find slam_project)/launch/car_bot_description.launch"/>

  <node name="car_bot_spawner" pkg="gazebo_ros" type="spawn_model" respawn="false" output="screen"
   args="-urdf -param robot_description -model car_bot"/>
</launch>
```

The launch file has the xml format as showed above. Note that world.launch file contain two wolds that are commented/uncommented depending on which world must be launched.

In addition to world.launch, we have the mapping.launch, used to load the rtabmap-ros package and the rtabmapviz or rviz, depending on the selection in the command line and the rviz.launch, used to load the rviz tool. Instead of rviz.launch, a different solution could be add the rviz command inside the mapping.launch and select it in command line. 

The final directory structure is showed in Fig. 9.

![alt text][image_8]

Fig. 9 - Directory structure

## Robot, map the world <a id='map'></a>

Putting all together, we can now start the mapping process.

Start launching the world and the robot model using the command

```sh
roslaunch slam_project world.launch
```

Now launch the rtabmap with rtabmaprviz tool, using the command

```sh
roslaunch slam_project mapping.launch
```

Finally, launch the teleop script to control the robot using the command

```sh
rosrun slam_project teleop
```

The mapping environment can be seen in Fig. 10

![alt text][image_9]

Fig. 10 - Robot mapping the user created world.

## Results <a id='result'></a>

