#!/bin/bash

echo $1
export ROS_MASTER_URI=$1
source devel/setup.bash 