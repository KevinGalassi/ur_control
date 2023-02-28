# DOCKERFILE ROS-BASE INSTALLATION

FROM ros:noetic-ros-core-focal

ENV DEBIAN_FRONTEND=noninteractive \
	LANG="en_US.UTF-8" \
    LANGUAGE="en_US.UTF-8" \
    LC_ALL="en_US.UTF-8" 

# install bootstrap tools
RUN apt-get update && apt-get install --no-install-recommends -y \
	apt-utils \
    bash-completion \
    build-essential \
    ca-certificates \
    curl \
    dirmngr \
    gdb \
    git \
    gnupg2 \
    keyboard-configuration \
    locales \
    lsb-release \
    python3-pip \
    software-properties-common \
    udev \
    vim \
    wget \
	python3-rosdep \
	python3-rosinstall \
	python3-vcstools \
    python3-catkin-tools \
    terminator \
    && rm -rf /var/lib/apt/lists/* \
    && locale-gen en_US.UTF-8; dpkg-reconfigure -f noninteractive locales


# bootstrap rosdep
RUN rosdep init && \
  rosdep update --rosdistro $ROS_DISTRO

# install ros packages
RUN apt-get update && apt-get install -y --no-install-recommends \
	ros-noetic-ros-base \
	&& rm -rf /var/lib/apt/lists/*

# Installation of EIGEN3
RUN apt update && apt install -y libeigen3-dev

# ROS Desktop packages
RUN apt install -y ros-noetic-moveit && \ 
	apt install -y ros-noetic-moveit-visual-tools

RUN apt update && apt install -y ros-noetic-ros-control && \
	apt install -y ros-noetic-ros-controllers && \
	apt install -y ros-noetic-geometry && \
	apt install -y ros-noetic-tf2-bullet


# Ros RQt/RViz/Gazebo packages

RUN apt update && apt install -y ros-noetic-rqt && \
	apt install -y ros-noetic-viz && \
	apt install -y ros-noetic-gazebo-ros-pkgs && \
	apt install -y ros-noetic-gazebo-ros-control


# UR packages 
RUN apt-get install -y ros-noetic-ur-client-library && \ 
   apt-get install -y ros-noetic-industrial-robot-status-interface && \ 
   apt-get install -y ros-noetic-ur-msgs && \ 
   apt-get install -y ros-noetic-scaled-joint-trajectory-controller && \ 
   apt-get install -y ros-noetic-speed-scaling-interface && \ 
   apt-get install -y ros-noetic-speed-scaling-state-controller && \ 
   apt-get install -y ros-noetic-pass-through-controllers && \ 
   apt-get install -y ros-noetic-gazebo-ros-control &&\
   apt-get install -y ros-noetic-robot && \
   apt-get install -y ros-noetic-trac-ik





RUN mkdir -p /ros_ws/src
WORKDIR /ros_ws

# Clone the Git repository into the workspace
RUN cd src && \
    git clone https://github.com/KevinGalassi/ur_control.git

RUN . /opt/ros/noetic/setup.sh && \
    catkin build


RUN export LC_NUMERIC="en_US.UTF-8"


# Start RVIZ when the container starts
RUN echo 'source devel/setup.bash' >> ~/.bashrc

#CMD roslaunch dual_ur_moveit_config demo.launch

CMD source devel/setup.bash 


