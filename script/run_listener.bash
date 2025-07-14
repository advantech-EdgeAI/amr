#!/bin/bash

#export ROS_DISCOVERY_SERVER="172.17.4.122:11811"
export ROS_DISCOVERY_SERVER="192.168.55.1:11811"
ros2 daemon stop
source /opt/ros/humble/setup.bash

python run_listener_node.py