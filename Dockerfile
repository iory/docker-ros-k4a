# Use the official ubuntu:18.04 image as the parent image
FROM nvidia/cuda:10.0-cudnn7-devel-ubuntu18.04
SHELL ["bash", "-c"]

ENV WORKSPACE /catkin_ws

ARG DEBIAN_FRONTEND=noninteractive

RUN if [ -e "/etc/apt/sources.list.d/cuda.list" ]; then \
      mv /etc/apt/sources.list.d/cuda.list /etc/apt/sources.list.d/cuda.list.save; \
    fi
RUN if [ -e "/etc/apt/sources.list.d/nvidia-ml.list" ]; then \
      mv /etc/apt/sources.list.d/nvidia-ml.list /etc/apt/sources.list.d/nvidia-ml.list.save; \
    fi
RUN apt update && apt install -y curl software-properties-common \
    && rm -rf /var/lib/apt/lists/* /var/cache/apt/archives/*

RUN apt update && apt install -y \
    pkg-config \
    ninja-build \
    doxygen \
    clang \
    gcc-multilib \
    g++-multilib \
    python3 \
    git-lfs \
    nasm \
    cmake \
    libgl1-mesa-dev \
    libsoundio-dev \
    libvulkan-dev \
    libx11-dev \
    libxcursor-dev \
    libxinerama-dev \
    libxrandr-dev \
    libusb-1.0-0-dev \
    libssl-dev \
    libudev-dev \
    mesa-common-dev \
    uuid-dev \
    libopencv-dev \
    expect \
    && rm -rf /var/lib/apt/lists/* /var/cache/apt/archives/*

RUN apt update && apt install -y vim \
    && rm -rf /var/lib/apt/lists/* /var/cache/apt/archives/*

ENV ACCEPT_EULA=Y
ENV DEBIAN_FRONTEND noninteractive
ENV DEBIAN_FRONTEND teletype
COPY ./install_k4a.exp /install_k4a.exp
RUN curl https://packages.microsoft.com/keys/microsoft.asc | apt-key add - \
    && apt-add-repository https://packages.microsoft.com/ubuntu/18.04/prod && apt update \
    && expect /install_k4a.exp \
    && rm -rf /var/lib/apt/lists/* /var/cache/apt/archives/*

RUN git clone https://github.com/microsoft/Azure-Kinect-Sensor-SDK
RUN cd Azure-Kinect-Sensor-SDK && mkdir build && cd build && cmake .. -GNinja && ninja

######### INSTALL ROS START ############
# install packages
RUN apt update && apt install -q -y \
    dirmngr \
    gnupg2 \
    lsb-release \
    && rm -rf /var/lib/apt/lists/* /var/cache/apt/archives/*

# setup keys
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -

# setup sources.list
RUN echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" > /etc/apt/sources.list.d/ros-latest.list

# install bootstrap tools
RUN apt update && apt install --no-install-recommends -y \
    python-rosdep \
    python-rosinstall \
    python-vcstools \
    && rm -rf /var/lib/apt/lists/* /var/cache/apt/archives/*

# setup environment
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8

# install ros packages
ENV ROS_DISTRO melodic
RUN apt update && apt install -y \
    ros-melodic-ros-core=1.4.1-0* \
    && rm -rf /var/lib/apt/lists/* /var/cache/apt/archives/*

######### INSTALL ROS END ############

RUN apt update && apt install --no-install-recommends -y \
    python-catkin-tools \
    && rm -rf /var/lib/apt/lists/* /var/cache/apt/archives/*

# This should be written in Azure_Kinect_ROS_Driver's package.xml
RUN apt update && apt install --no-install-recommends -y \
    ros-melodic-joint-state-publisher \
    ros-melodic-robot-state-publisher \
    && rm -rf /var/lib/apt/lists/* /var/cache/apt/archives/*

WORKDIR ${WORKSPACE}
RUN git clone https://github.com/microsoft/Azure_Kinect_ROS_Driver ${WORKSPACE}/src/Azure_Kinect_ROS_Driver

# bootstrap rosdep
RUN apt update && rosdep init && rosdep update \
    && rosdep install --from-paths -i -r -y src \
    && rm -rf /var/lib/apt/lists/* /var/cache/apt/archives/*

RUN mv /bin/sh /bin/sh_tmp && ln -s /bin/bash /bin/sh
RUN source /opt/ros/${ROS_DISTRO}/setup.bash; catkin build -DCMAKE_BUILD_TYPE=Release
RUN rm /bin/sh && mv /bin/sh_tmp /bin/sh

# nvidia-container-runtime
ENV NVIDIA_VISIBLE_DEVICES \
  ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
  ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

RUN apt update && apt install --no-install-recommends -y \
    # https://stackoverflow.com/questions/49333582/portaudio-library-not-found-by-sounddevice
    libportaudio2 \
    python-pip \
    ros-melodic-openni2-launch \
    ros-melodic-jsk-tools \
    ros-melodic-image-proc \
    ros-melodic-depth-image-proc \
    ros-melodic-jsk-rviz-plugins \
    ros-melodic-compressed-depth-image-transport \
    ros-melodic-compressed-image-transport\
    ros-melodic-audio-common-msgs \
    ros-melodic-usb-cam \
    && rm -rf /var/lib/apt/lists/* /var/cache/apt/archives/*

RUN pip install numpy
RUN pip install sounddevice

COPY ./launch/kinectdriver_nodelet.launch /kinectdriver_nodelet.launch
COPY ./node_scripts/audio_capture.py /catkin_ws/src/Azure_Kinect_ROS_Driver/audio_capture.py
COPY ./launch/usb_cam.launch /usb_cam.launch

CMD ["bash"]
