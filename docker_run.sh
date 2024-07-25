#!/bin/bash
#set -e

CONTAINER_NAME="my_bundle_adjuster"
KITTI_DATASET_PATH="/home/hamdaan/Datasets/KITTI/data_odometry_gray/dataset/sequences/00"

# Removing the container incase it is present
docker rm $CONTAINER_NAME

# Creating the docker container
docker create \
--privileged -it \
--network host -e DISPLAY=$DISPLAY \
-v /tmp/.X11-unix:/tmp/.X11-unix \
-v ${PWD}:/home/bundle_adjustment_on_manifold \
-v $KITTI_DATASET_PATH:/home/kitti \
--name $CONTAINER_NAME bundle_adjuster

# Start the container
docker start my_bundle_adjuster