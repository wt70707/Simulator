#!/bin/bash
# Script to install Spiri Simulator

sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu precise main" > /etc/apt/sources.list.d/ros-latest.list'

wget https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -O - | sudo apt-key add -

sudo apt-get update

sudo apt-get install ros-hydro-desktop-full

sudo apt-get install ros-hydro-hector-quadrotor

sudo apt-get install ros-hydro-ros-controllers

sudo apt-get install ros-hydro-joy


# Configuring ros workspace

echo "source /opt/ros/hydro/setup.bash" >> ~/.bashrc
source ~/.bashrc

mkdir -p ~/catkin_ws/src

cd ~/catkin_ws/src

catkin_init_workspace

echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc

# Copy Spiri code

cd ~/catkin_ws/src

git clone https://github.com/Pleiades-Spiri/Simulator.git -b alpha-release

cd ..

catkin_make

