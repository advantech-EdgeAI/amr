#!/bin/bash

CONTAINER_NAME="x86_container"
#DOCKER_IMAGE="remembr_demo:x86"
#DOCKER_IMAGE="remembert_agent:r0.2.0"
DOCKER_IMAGE="remembr_agent:r0.2.0"
RELEASE="/release"
STATUS=$(docker inspect -f '{{.State.Status}}' "$CONTAINER_NAME" 2>/dev/null)

#        --shm-size 14G \
#        -v ./release_310_x86_64:$RELEASE \
if [ "$STATUS" = "running" ]; then
    docker exec -it $CONTAINER_NAME bash
elif [ "$STATUS" = "exited" ]; then
    docker start $CONTAINER_NAME
    docker exec -it $CONTAINER_NAME bash
else
    docker run \
        -it \
        --rm \
        --name $CONTAINER_NAME \
        --ipc host \
        --network host \
        --gpus all \
        --shm-size 8G \
        -e ROS_IP=$(hostname -I | awk '{print $1}') \
        -e release=$RELEASE \
        -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
        -v ./release:$RELEASE \
        -v ./script:/script \
        -w /script \
        $DOCKER_IMAGE
fi
