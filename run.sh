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
       --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
       -it iory/k4a:melodic /bin/bash -i -c \
       "source /catkin_ws/devel/setup.bash && export ROS_MASTER_URI=http://localhost:11311 && roslaunch /kinectdriver_nodelet.launch camera:=${1:k4a} body_tracking_enabled:=false depth_mode:=WFOV_2X2BINNED color_resolution:=2160P fps:=30 point_cloud:=false rgb_point_cloud:=false publish_tf:=false color_format:=jpeg"
xhost +local:docker
