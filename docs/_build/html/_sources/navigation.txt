Navigation with Spiri
=====================

Spiri comes with higher level robot functions which can be used as building
blocks when creating your robot application. One of these building blocks is
the Moveit navigation stack, which allows the robot to navigate from place to
place within the environment.

For more complete documentation of the individual components of the navigation
stack, see the documentation on the
`Moveit wiki <http://moveit.ros.org/documentation/tutorials/>`_.


Starting the Simulator with moveit
-----------------------------------

::

    >$ roslaunch spiri_description move_it_spiri.launch

Run the controller which takes in a trajectory, calculates and sends velocity to Spiri.

::

    >$ rosrun action_controller actioncontroller.py

Wait for controllers and planning algorithms to come up. When you see the message "All is well! Everyone is happy! You can start planning now!" in move_it_spiri.launch terminal it means you can start sending goals.

Using RVIZ to send goals
---------------------------
Launch Rviz

::

    >$ rosrun rviz rviz



We can send goals using Moveit plugin for Rviz.

.. figure:: images/Rviz-spiri-goals.png
   :width: 100%
   :align: center
   :figclass: align-centered

Note that we have used **PRMKConfig** planner. In the drop down we have only included two planners as we have tested them with Spiri.

.. figure:: images/rviz-spiri-workspace.png
   :width: 100%
   :align: center
   :figclass: align-centered

Note that we have increase the **workspace** size. This is required for sending goals that are far.

You can set the goal state using interactive markers. After setting the goal state, press **plan and execute** and Spiri should move towards the goal state.

For more information refer to the link `Goals using moveit <http://moveit.ros.org/wiki/PR2/Rviz_Plugin/Quick_Start>`_.


Using API to send goals
---------------------------

Sending commands using Python
+++++++++++

There are examples scripts located in **examples** folder. We have made it easy to send goals using our API.

.. code-block:: python

	from planner_request import get_state
	# create an object for sending commands to Spiri
	spiri=get_state.Staterobot()
	# send goals with respect to start position
	spiri.send_goal_relative(0,0,1)
	# send goals with respect to world
	#spiri.send_goal(0,0,1)

Reading Sensor Data
---------------------------

Using ROS
+++++++++++

We have followed the ROS convention. All the sensor data are publised on ROS topics. 

::

	>$ rostopic list
	>$ rostopic echo /raw_imu

For more information on `ROS topics <http://wiki.ros.org/Topics>`_.


Using API
+++++++++++

There are examples scripts located in **examples** folder. We have made it easy to read sensor data using our API.

.. code-block:: python 

	from planner_request import get_state
	# create an object to communicate with Spiri
	spiri=get_state.Staterobot()
	# get orientation from IMU in euler format
	imu=spiri.get_orientation_imu('euler')
	print 'Roll',imu.x,'Pitch',imu.y,'Yaw',imu.z

	# get orientation from IMU in Quaternion format
	imu=spiri.get_orientation_imu('quat')
	print imu.x,imu.y,imu.z,imu.w

	#get GPS latitude, longitude and altitude
	gps=spiri.get_gps_data()
	print 'Latitude',gps.latitude,'Longitude',gps.longitude,'height',gps.altitude

	#get velocity from GPS
	gps_vel=spiri.get_gps_vel()
	print gps_vel.x,gps_vel.y,gps_vel.z
	# get altitude from pressure sensor
	pressure_height=spiri.get_height_pressure()
	print 'height from pressure sensor is',pressure_height

	# get altitude from altimeter
	altimeter_height=spiri.get_height_altimeter()
	print 'height from altimeter is',altimeter_height


	






