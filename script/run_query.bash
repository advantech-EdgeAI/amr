#!/bin/bash

QUERY="where is water dispenser"

if [ -z "$ROS_DISCOVERY_SERVER" ]; then
#    export ROS_DISCOVERY_SERVER="172.17.4.122:11811"
    export ROS_DISCOVERY_SERVER="192.168.55.1:11811"
    ros2 daemon stop
    source /opt/ros/humble/setup.bash
fi

ros2 topic pub -1 /speech std_msgs/msg/String "data: $QUERY"