#!/bin/bash

echo "hello This will get rid of Spiri's Simulator. If you have any files that are in the Simulator folder please back it up"

read -p "Do you want to continue? [Y/n]" yn


case $yn in
	[Yy]* )

		sudo apt-get install ros-hydro-moveit-full ros-hydro-joystick-drivers
		rm -r ~/catkin_ws/src/Simulator
		# lower the version number
		rm ~/catkin_ws/src/Simulator-1.0.tar.gz

		#get the new Simulator

		cd ~/catkin_ws/src
		# bump the version number
		wget https://raw.github.com/Pleiades-Spiri/Spiri_Public/installation_simulator/Simulator-1.0.tar.gz

		tar -zxvf Simulator-1.0.tar.gz 

		mv Simulator-1.0 ./Simulator

		cd ..

		#Build the system

		catkin_make;;
	[Nn]* ) exit;;
esac


