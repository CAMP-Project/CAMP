# Coordinated Autonomous Mapping & Pathfinding

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
