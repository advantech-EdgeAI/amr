## reference
https://github.com/addpipe/simple-recorderjs-demo

## initialize
openssl req -x509 -nodes -days 365 -newkey rsa:2048 -keyout key.pem -out cert.pem
sudo apt-get install ros-humble-rosbridge-server

## activate
# terminal 1
python web/main.py
# terminal 2
python python/asr_node_lite.py
# terminal 3
source /opt/ros/humble/setup.bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml address:=0.0.0.0 ssl:=true certfile:=/remembr/web/cert.pem keyfile:=/remembr/web/key.pem