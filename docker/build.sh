#!/bin/bash

echo "Building docker"

IMAGE_NAME=mad-icp

# docker build --build-arg UNAME=$(whoami) --build-arg UID=$(id -u) --build-arg GID=$(id -g) -t ${IMAGE_NAME} .
docker build -t ${IMAGE_NAME} .

echo "Done!"
