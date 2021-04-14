#!/bin/bash

source devel/setup.bash 
# TODO: Add additinal laaunch commands to this startup file

export TURTLEBOT3_MODEL='burger'
roslaunch brian brian.launch
