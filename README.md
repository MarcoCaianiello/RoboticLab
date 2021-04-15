# Caianiello - Morra Exam Project
## Getting Started
The following is a project developed for the Robotics Lab course at Federico II, University of Naples.

The goal is to develop a robotic exploration system composed by two mobile ground robots. One of them have to explore the environment and looking for seven QR markers whose id is a number going from 1 to 7. The other one has to navigate the spots in front of the markers in a given sequence. The sequence must be dynamically specified by the user before starting the navigation.

Two demo videos and a technical report are located in this repository to better understand its funtionalities.

## Requirements and installation
The project is developed on a full desktop installation of [ROS Melodic](http://wiki.ros.org/melodic/Installation/Ubuntu) on Ubuntu 18.04.4 LTS.

The simulations are carried out in Gazebo Sim Software, available at: http://gazebosim.org/

The mobile ground robots chosen are the ones from the ROBOTIS Turtlebot's family (https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/). Turtlebots require three different packages:
 - [turtlebot3](https://github.com/ROBOTIS-GIT/turtlebot3)
 - [turtlebot3_msgs](https://github.com/ROBOTIS-GIT/turtlebot3_msgs)
 - [turtlebot3_simulation](https://github.com/ROBOTIS-GIT/turtlebot3_simulations)

All of these packages are already included, however, for the latest version, it is recommended to clone them from the official repositories.

For the QR marker detection, ViSP library is needed. It is available at the following:

[ViSP stack for ROS](https://github.com/lagadic/vision_visp)

    $ sudo apt-get install ros-melodic-visp

The creation of a map of the environment and the navigation are demanded to the [gmapping](http://wiki.ros.org/gmapping) algorithm and the [move_base](http://wiki.ros.org/move_base) package. Furthermore, in order to retrieve localization, [amcl](http://wiki.ros.org/amcl) algorithm has been adopted.

    $ sudo apt-get install ros-melodic-gmapping
    $ sudo apt-get install ros-melodic-amcl
    $ sudo apt-get install ros-melodic-move-base
    $ sudo apt-get install ros-melodic-move-base-msgs
    $ sudo apt-get install ros-melodic-map-server
    $ sudo apt-get install ros-melodic-dwa-local-planner



## Installation
To allow the SLAM algorithm to save the map and the Move Base to read it, the user have to specify a valid path in two different files. The first file is exam/config/param.yaml and the variable to be edited is 'map_abs_path'. After that, it is necessary to align this to the one specified in exam/launch/navigation.launch under the 'map_file' argument.

## Run - Phase 1: exploration
To start the simulated scene, a proper launchfile is provided:

    roslaunch exam start.lauch world:=w1
 
 This file starts Gazebo and spawns both the turtlebots. 
 To spawn a different world setup, it is possible to pass a different world argument (w1, w2).
 
 For the visual servoing, the detection file has to be launched:
 
    roslaunch exam detection.lauch

 In order to start the phase, exploration has to be launched:
 
    roslaunch exam exploration.lauch

When all the markers have been found a proper message will be displayed both in the start.launch and detection.launch windows. The map is then saved to the formerly specified path.

It is mandatory to wait for the robot to return to base before proceding to the next phase.

It is then possible to close exploration.launch window.
ViSP nodes will be automatically closed to increase performance.
Do not kill the start.launch as it holds the simulation and do not kill detection node as it will send the path message to the Move Base client in the next phase.


## Run - Phase 2: navigation
The Move Base server can now be initialized by:

    roslaunch exam navigation.launch

Then, to make the second robot navigate the 7 spots (in order), the Move Base client has to be run:

    rosrun exam navigation
    
The navigation sequence can also be specified by input by running (for example):

    rosrun exam navigation 4 5 6
    
The outcome can be monitored both on the Gazebo simulation and on the terminal.

Once every spot has been visited, another navigation sequence could be initialized by running a new navigation node.
