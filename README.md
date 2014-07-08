Simulator
=========
-------------------------------------------------------------------
This branch contains the alpha release of the simulator.

It will provide the following functionalities-:

1. Spawn Spiri in Gazebo with a choice of a world

2. Publish sensor data on individual topics.

3. Control Spiri in the simulator by a joystick.

-------------------------------------------------------------------

-------------------------------------------------------------------
Installation instructions

1. Install ROS

http://wiki.ros.org/hydro/Installation/Ubuntu

Install the ros-desktop-full package as it contains Gazebo and other programs required for the simulator.

2. Install Gazebo sensor plugins

sudo apt-get install ros-hydro-hector-quadrotor

3. Create a ros workspace

http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment

After this step there should be a folder called as catkin_ws

4. Get the Simulator

cd catkin_ws/src

git clone https://github.com/Pleiades-Spiri/Simulator.git -b alpha-release

cd ..

catkin_make

This command will build the code required for the Simulator.

-------------------------------------------------------------------

-------------------------------------------------------------------


Commands

To launch Spiri in an empty world

roslaunch spiri_description spiri_empty_world.launch

To launch Spiri in Citadel Hill

roslaunch spiri_description spiri_citadel.launch

Script to process the Depth Map

ROS_NAMESPACE=stereo rosrun stereo_image_proc stereo_image_proc _approximate_sync:=True _queue_size:=10

View the Depth Map

rosrun image_view stereo_view stereo:=/stereo image:=image_rect_color _approximate_sync:=True _queue_size:=10

-------------------------------------------------------------------

-------------------------------------------------------------------
To visualize the robot and point cloud data in rviz

roscd spiri_description

rosrun rviz rviz -d spiri_pointcloud.rviz

-------------------------------------------------------------------



