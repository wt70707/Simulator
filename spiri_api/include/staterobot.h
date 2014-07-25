#ifndef STATEROBOT_H
#define STATEROBOT_H
#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PointStamped.h"
#include "sensor_msgs/NavSatFix.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "hector_uav_msgs/Altimeter.h"
#include "ros/topic.h"
#include <vector>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
class Staterobot
{
    std::vector<double> group_variable_values;
    moveit_msgs::RobotState start_state;
    geometry_msgs::Transform transform;
    geometry_msgs::Pose current_state;
    bool success_plan;
    bool success_execution;

private:

public:
    Staterobot();

    geometry_msgs::Quaternion get_imu();
    geometry_msgs::Pose get_state();
    float get_height_pressure();

    sensor_msgs::NavSatFixConstPtr get_gps_data();
    geometry_msgs::Vector3 get_gps_vel();
    float get_height_altimeter();
    bool send_goal(float x,float y,float z,bool relative);
    sensor_msgs::ImuConstPtr imu;
    nav_msgs::OdometryConstPtr odom;
    geometry_msgs::PointStampedConstPtr pressure;
    sensor_msgs::NavSatFixConstPtr gps;
    geometry_msgs::Vector3StampedConstPtr gps_vel;
    hector_uav_msgs::AltimeterConstPtr altimeter;

};

#endif // STATEROBOT_H
