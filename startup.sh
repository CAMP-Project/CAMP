#!/bin/bash

source devel/setup.bash 
# TODO: Add additinal launch commands to this startup file

export TURTLEBOT3_MODEL='burger'
sudo chmod 777 /dev/ttyACM0
sudo chmod 777 /dev/ttyACM1
roslaunch brian brian.launch
