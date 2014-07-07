Simulator
=========

To launch spiri in Gazebo run the following command

roslaunch spiri_description spiri_empty_world.launch


For depth map-:

ROS_NAMESPACE=stereo rosrun stereo_image_proc stereo_image_proc _approximate_sync:=True _queue_size:=10

To view depth map-:

rosrun image_view stereo_view stereo:=/stereo image:=image_rect_color _approximate_sync:=True _queue_size:=10


To launch spiri in citadel hill-:

roslaunch spiri_description spiri_citadel.launch 
