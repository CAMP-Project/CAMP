# Coordinated Autonomous Mapping & Pathfinding
This project was put together by Sean Carda, Manpreet Singh, and Tyler Pigott with help from Dr. Michael McCourt and a foundation from the FireForce team as a part of the Electrical Engineering Program at the Univerity of Washington Tacoma and continued into the summer after graduation as a research project. Our goal was to get multiple autonomous Turtlebot3 robots to work together to map a space, and we got pretty darn close.

The final result of our project wasseveral useful bits of code that could combine with more time and development to reach the project goal. Packages include one that accepts navigation commands to navigate to points, several options for generating points, a Kalman filter to help smooth time-of-flight sensor data, the Multimaster Arbitrator to facilitate inter-robot communication, and some packages for transforming maps. A more detailed account of project contents is below.

## Project Depenendencies
* [Multimaster FKIE](http://wiki.ros.org/multimaster_fkie)
* [Python2 version of OpenCV](https://github.com/opencv/opencv)
* ROS Packages listed in the ROS Installation Crash Course Document

The two Documents in this repository contain information on Getting Started with this project and even getting the specific ROS packages installed to be able to build the software stack.
* [Getting Started Doc.odt](https://github.com/CAMP-Project/CAMP/raw/master/Getting%20Started%20Doc.odt)
* [ROS Installation Crash Course.odt](https://github.com/CAMP-Project/CAMP/raw/master/ROS%20Installation%20Crash%20Course.odt)

## How to build project

Before building this project, be sure to install the project dependencies listed above. Once setup, run the following commands in the terminal.

```
git clone https://github.com/CAMP-Project/CAMP.git

cd CAMP

git submodule update --init

catkin_make
```

## Project Contents
Here are our interesting or new contributions to the project. There are more packages in the srcfolder, but some of them come with the turtlebot or where inherited from previous groups.
### [Brian](https://github.com/CAMP-Project/CAMP/tree/master/src/brian) - The Multimaster Arbitrator
The Brian folder contains code related to the multimaster arbitrator. The arbitrator helps package topics up in a way where other robots can use the data.

Launch files:
* brian.launch - launches turtlebot bringup and map.

Source code:
* MMArbitrator.cpp - its the arbitrator, breif description

### [Camp](https://github.com/CAMP-Project/CAMP/tree/master/src/camp) - Goto, Pathfinding, and Maps.
#### [Camp Goto](https://github.com/CAMP-Project/CAMP/tree/master/src/camp/camp_goto) - Navigation to a Point
#### [Camp Map](https://github.com/CAMP-Project/CAMP/tree/master/src/camp/camp_map) - Mappping and Map Transforms
#### [Camp Multiagent](https://github.com/CAMP-Project/CAMP/tree/master/src/camp/camp_multiagent) - Working together
#### [Camp Pathfinding](https://github.com/CAMP-Project/CAMP/tree/master/src/camp/camp_pathfinding) - Finding a Way
### [Coop Test](https://github.com/CAMP-Project/CAMP/tree/master/src/coop_test) - Testing Cooperation
### [Kalman Filter](https://github.com/CAMP-Project/CAMP/tree/master/src/kalman_filter) - Smoothing Time-of-Flight Data
### [Lidar Listener](https://github.com/CAMP-Project/CAMP/tree/master/src/lidar_listener) - Our First Sensor Test
### [Roomba Navigation](https://github.com/CAMP-Project/CAMP/tree/master/src/roomba_navigation) - Minimum Deliverable Product
### [TF Test Environment](https://github.com/CAMP-Project/CAMP/tree/master/src/tf_test_environment) - Figuring Out Transforms
