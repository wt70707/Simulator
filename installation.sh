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
	;;
	
esac

wget https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -O - | sudo apt-key add -

sudo apt-get update
# Install all the ROS required packages

sudo apt-get install ros-hydro-desktop-full ros-hydro-hector-quadrotor ros-hydro-ros-controllers ros-hydro-joy ros-hydro-moveit-full ros-hydro-joystick-drivers

# Configuring ros workspace

echo "source /opt/ros/hydro/setup.bash" >> ~/.bashrc

echo "Restart the terminal and run installation_simulator.sh"




