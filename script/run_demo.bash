#!/bin/bash

export ROS_DISCOVERY_SERVER="172.17.4.122:11811"
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ros2 daemon stop
source /opt/ros/humble/setup.bash

ros2 launch rosbridge_server rosbridge_websocket_launch.xml address:=0.0.0.0 ssl:=true certfile:=/script/web/cert.pem keyfile:=/script/web/key.pem &
python run_asr_node.py &

python web/main.py
