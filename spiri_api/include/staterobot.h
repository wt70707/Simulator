#ifndef STATEROBOT_H
#define STATEROBOT_H
#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PointStamped.h"
#include "sensor_msgs/NavSatFix.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/image_encodings.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "hector_uav_msgs/Altimeter.h"
#include "ros/topic.h"
#include <ros/callback_queue.h>
#include <vector>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <opencv2/highgui/highgui.hpp>
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
    cv::Mat left_image;
    ros::CallbackQueue image_queue;

private:

public:
    Staterobot();

    geometry_msgs::Quaternion get_imu();
    cv::Mat get_left_camera();
    void image_callback(const sensor_msgs::ImageConstPtr &);
    geometry_msgs::Pose get_state();
    float get_height_pressure();

    sensor_msgs::NavSatFixConstPtr get_gps_data();
    geometry_msgs::Vector3 get_gps_vel();
    float get_height_altimeter();
    bool send_goal(float x,float y,float z,bool relative);
    cv::Mat get_left_image();

    sensor_msgs::ImuConstPtr imu;
    nav_msgs::OdometryConstPtr odom;
    geometry_msgs::PointStampedConstPtr pressure;
    sensor_msgs::NavSatFixConstPtr gps;
    geometry_msgs::Vector3StampedConstPtr gps_vel;
    hector_uav_msgs::AltimeterConstPtr altimeter;
    sensor_msgs::ImageConstPtr img;
    cv_bridge::CvImagePtr cv_ptr;
    image_transport::Subscriber sub;


};

#endif // STATEROBOT_H
