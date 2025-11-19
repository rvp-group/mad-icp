#!/bin/bash
IMAGE_NAME=mad-icp

xhost +

docker run \
  -ti \
  -it \
  --rm \
  --env="DISPLAY" \
  --shm-size 24G \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  --privileged \
  --network host \
  -e DISPLAY=$DISPLAY \
  -e QT_X11_NO_MITSHM=1 \
  ${IMAGE_NAME} \
  bash -c /bin/bash
