#!/bin/bash

CONTAINER_NAME="vllm_container"
#DOCKER_IMAGE="remembr_demo:vllm"
#DOCKER_IMAGE="remember_vllm:r0.1.0"
DOCKER_IMAGE="remembr_vllm:r0.2.0"
RELEASE="/release"
STATUS=$(docker inspect -f '{{.State.Status}}' "$CONTAINER_NAME" 2>/dev/null)

##        -v ./release_312_aarch64:$RELEASE \
if [ "$STATUS" = "running" ]; then
    docker exec -it $CONTAINER_NAME bash
elif [ "$STATUS" = "exited" ]; then
    docker start $CONTAINER_NAME
    docker exec -it $CONTAINER_NAME bash
else
    # add this line to add camera
    # for example
        # --device /dev/video4 \
    docker run \
        -it \
        --rm \
        --name $CONTAINER_NAME \
        --privileged \
        --network host \
        -e ROS_IP=$(hostname -I | awk '{print $1}') \
        -e release=$RELEASE \
        -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
        -v ./release:$RELEASE \
        -v ./script:/script \
        -w /script \
        $DOCKER_IMAGE
fi
