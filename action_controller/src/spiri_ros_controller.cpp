#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "trajectory_msgs/MultiDOFJointTrajectory.h"
#include "geometry_msgs/Pose.h"
#include <boost/bind.hpp>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <tf/transform_datatypes.h>
#include <spiri_motion_primitives/SpiriMoveToAction.h>
class Spiri_ros_controller
{

public:
Spiri_ros_controller()
{
ros::NodeHandle n;
ros::Subscriber sub=n.subscribe<trajectory_msgs::MultiDOFJointTrajectory>("command",1,boost::bind(&Spiri_ros_controller::vel_callback,this, _1));

}

void vel_callback(const trajectory_msgs::MultiDOFJointTrajectoryConstPtr& msg)
{
    ROS_INFO("Recieved the trajectory");
    actionlib::SimpleActionClient<spiri_motion_primitives::SpiriMoveToAction>ac("spiri_motion_primitives",true);
    ROS_INFO("Waiting for action server to start.");
    // wait for the action server to start
    ac.waitForServer(); //will wait for infinite time

    ROS_INFO("Action server started, sending goal.");
    // send a goal to the action
    spiri_motion_primitives::SpiriMoveToGoal goal;


    goal.pose.header.stamp = ros::Time::now();
    goal.pose.header.frame_id = "base_link";
    goal.pose.pose.position.x = 0;//msg->points[1].transforms[0].translation.x;
    goal.pose.pose.position.y = 0;//msg->points[1].transforms[0].translation.y;
    goal.pose.pose.position.z = 1;//msg->points[1].transforms[0].translation.z;

    goal.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.0, 0.0);

    goal.speed = 1.0;
    goal.tolerance = 0.1;

    goal.use_distance_from_ground = false;

    ac.sendGoal(goal);
    bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));

      if (finished_before_timeout)
      {
          actionlib::SimpleClientGoalState state = ac.getState();
          ROS_INFO("Action finished: %s",state.toString().c_str());
      }
      else
      {
          ac.cancelGoal();
          ROS_INFO("Action did not finish before the time out.");
      }

      //exit
      //return 0;


}

};








int main(int argc, char **argv)
{
  ros::init(argc, argv, "spiri_command_listener");

  Spiri_ros_controller controller;
  ros::spin();

  return 0;
}
