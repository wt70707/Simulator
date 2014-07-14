#!/bin/bash
# Script to install Spiri Simulator
VER=$(lsb_release -sr)

case "$VER" in 
	"12.04") 
	sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu precise main" > /etc/apt/sources.list.d/ros-latest.list'
	;;
	"12.10") 
	sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu quantal main" > /etc/apt/sources.list.d/ros-latest.list'
	;;
	"13.04")
	sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu raring main" > /etc/apt/sources.list.d/ros-latest.list'


	
esac

wget https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -O - | sudo apt-key add -

sudo apt-get update
# Install all the ROS required packages

sudo apt-get install ros-hydro-desktop-full ros-hydro-hector-quadrotor ros-hydro-ros-controllers ros-hydro-joy

# Configuring ros workspace

echo "source /opt/ros/hydro/setup.bash" >> ~/.bashrc
source ~/.bashrc

mkdir -p ~/catkin_ws/src

cd ~/catkin_ws/src

catkin_init_workspace

cd ..

catkin_make

echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc

sudo rosdep init

rosdep update
# Download Spiri simulator

cd ~/catkin_ws/src

wget https://raw.github.com/Pleiades-Spiri/Spiri_Public/installation_simulator/Simulator-1.0.tar.gz

tar -zxvf Simulator-1.0.tar.gz 

mv Simulator-1.0 ./Simulator

cd ..

#Build the system

catkin_make

# Set Gazebo Model Path

echo "export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/catkin_ws/src/Simulator/spiri_description/models" >> ~/.bashrc





