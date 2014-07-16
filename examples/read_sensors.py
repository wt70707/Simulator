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
