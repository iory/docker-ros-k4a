#!/bin/bash

echo "waiting for roscore to come up"
. /opt/ros/melodic/setup.bash
rossetip
rossetmaster 133.11.216.211

ret=`rosnode list`
for i in `seq 0 1000`
do
  if [ "$ret" != '' ]
  then
    break
  fi
  ret=`rosnode list`
  sleep 1
  echo "Waiting for roscore $i/1000"
done

if [ "$ret" = '' ]
then
  echo "Roscore is down"
  exit 1
fi

. /home/applications/ros/melodic/devel/setup.bash
rossetip
rossetmaster 133.11.216.211

export DISPLAY=:1
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
       -i iory/docker-ros-k4a:latest /bin/bash -i -c \
       "source /catkin_ws/devel/setup.bash && roslaunch /kinectdriver_nodelet.launch camera:=${1:k4a} body_tracking_enabled:=true depth_mode:=WFOV_2X2BINNED color_resolution:=1440P fps:=30 point_cloud:=false rgb_point_cloud:=false color_format:=jpeg"
xhost +local:docker
