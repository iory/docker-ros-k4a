#!/bin/bash

xhost +local:root
docker run --rm \
       --gpus all \
       --privileged \
       --volume="/dev:/dev" \
       --name "kinect" \
       --net=host \
       --env="DISPLAY" \
       --env="QT_X11_NO_MITSHM=1" \
       --env="ROS_IP" \
       --env="ROS_MASTER_URI" \
       --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
       -it iory/docker-ros-k4a:latest /bin/bash -i -c \
       "source /catkin_ws/devel/setup.bash && roslaunch /kinectdriver_nodelet.launch camera:=${1:k4a} body_tracking_enabled:=true depth_mode:=WFOV_2X2BINNED color_resolution:=1440P fps:=30 color_format:=jpeg point_cloud:=false rgb_point_cloud:=false"
xhost +local:docker
