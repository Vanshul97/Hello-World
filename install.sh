#! /bin/bash

sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt update
sudo apt install ros-melodic-desktop-full
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
source ~/.bashrc
sudo apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential
sudo apt install python-rosdep
sudo rosdep init
rosdep update
sudo apt-get install ros-melodic-moveit
sudo apt-get install python-catkin-tools
sudo apt-get install libgflags-dev
sudo apt-get install ros-$ROS_DISTRO-rosparam-shortcuts
sudo apt-get install ros-melodic-ros-control ros-melodic-ros-controllers
sudo apt install python3 python3-pip
sudo apt install ros-melodic-rosparam-shortcuts
pip3 install odrive
pip3 install rospkg
catkin init
catkin build
source install/setup.bash
