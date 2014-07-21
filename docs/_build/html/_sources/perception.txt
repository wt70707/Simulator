Perception with Spiri
=====================

Spiri comes with higher level robot functions which can be used as building
blocks when creating your robot application. One of the key building blocks of
robot applications is perception. Spiri comes with a complete sensor suite for
perceiving the world around it, as well as software tools to carry out basic
perception tasks.

Using Camera Data
-----------------
Spiri comes equipped with three cameras, two front facing cameras and one bottom camera.

The two front facing cameras gives us steroscopic vision which helps us in perceiving depth and creating 3-d movies.

This section provides an overview of how to access the camera data and run depth algorithms using ROS.

The frontal cameras on Spiri is capable of producing both 2d and 3d images.
These are available over ROS topics, of both ``sensor_msgs/Image`` and
``sensor_msgs/PointCloud2``.


View the camera images
+++++++++++

View the front left camera

::

	>$ rosrun image_view image_view image:=/stereo/left/image_raw

View the front right camera

::

	>$ rosrun image_view image_view image:=/stereo/right/image_raw

View the bottom facing camera

::

	>$ rosrun image_view image_view image:=/downward_cam/camera/image


Depth Map Proceesing
+++++++++++

Run the depth map algorithm

::
	
	>$ ROS_NAMESPACE=stereo rosrun stereo_image_proc stereo_image_proc _approximate_sync:=True _queue_size:=10


View the depth map

::

	>$ rosrun image_view stereo_view stereo:=/stereo image:=image_rect_color _approximate_sync:=True _queue_size:=10
	

.. figure:: images/depth_map_simulator.png
   :width: 100%
   :align: center
   :figclass: align-centere

Access camera data using API
-------------------------

Coming Soon

TODO. 


