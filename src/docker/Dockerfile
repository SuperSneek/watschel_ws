# Docker Image for development of solo8 software
FROM ubuntu:24.04

ARG ROS_DISTRO=jazzy

### ROS Setup ###
# locales
RUN apt-get update
RUN apt-get install -y locales
RUN locale-gen en_US en_US.UTF-8
RUN update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
RUN export LANG=en_US.UTF-8
RUN apt-get -y install apt-utils
RUN sh -c "echo 'debconf debconf/frontend select Noninteractive' | debconf-set-selections"
RUN apt-get install -y -q

# universe
RUN apt-get install -y software-properties-common curl
RUN add-apt-repository universe

# PyEnv for Python Version Management
RUN curl https://pyenv.run | bash

# ROS GPG key
RUN apt-get update
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# ROS repositories
RUN sh -c 'echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null'
RUN apt-get update && apt-get upgrade -y

# ROS dev tools
RUN apt-get install -y ros-dev-tools
RUN apt-get install -y ros-$ROS_DISTRO-desktop ros-$ROS_DISTRO-joint-state-publisher-gui ros-$ROS_DISTRO-robot-state-publisher ros-$ROS_DISTRO-xacro
RUN apt-get install -y ros-$ROS_DISTRO-gz-tools-vendor ros-$ROS_DISTRO-gz-sim-vendor
RUN apt-get install -y python3 python3-pip python3-colcon-common-extensions

WORKDIR "/home"

### Solo8 Setup ###
RUN git clone https://github.com/Nordegraf/brunhilde_ws.git --recursive
RUN apt-get install -y libeigen3-dev pybind11-dev doxygen ros-jazzy-eigenpy libyaml-cpp-dev libboost-all-dev

# Python packages
RUN pip install numpy --upgrade
RUN pip install xacro pin

ENV DISPLAY :0

# Entrypoint
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
