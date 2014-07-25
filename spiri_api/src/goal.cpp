#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <iostream>
int main(int argc, char **argv)
{
    ros::init(argc, argv, "move_group_interface_demo", ros::init_options::AnonymousName);
    // start a ROS spinning thread
    ros::NodeHandle node_handle;

    ros::AsyncSpinner spinner(1);
    spinner.start();
    moveit::planning_interface::MoveGroup group("spiri");

    group.setPlannerId("PRMkConfigDefault");
    geometry_msgs::Pose target_pose;
    target_pose.position.z=1.0;
    target_pose.orientation.w=1.0;
    //group.setPoseTarget(target_pose);
    //std::cout<<group.getJointValueTarget();
    double a=1.0;
    std::vector<double> group_variable_values;
    group.getCurrentState()->copyJointGroupPositions(group.getCurrentState()->getRobotModel()->getJointModelGroup(group.getName()), group_variable_values);
    for(auto i = group_variable_values.begin(); i != group_variable_values.end(); ++i)
      {
          std::cout << *i;
      }
    //std::cout<<group_variable_values;
    group_variable_values[2] = 1.0;
    group.setJointValueTarget(group_variable_values);
    //group.setJointValueTarget(target_pose);
    moveit::planning_interface::MoveGroup::Plan my_plan;
    bool success = group.plan(my_plan);
    // std::cout<<success;
    if(success==1)
    {
        ROS_INFO("Going to execute");
        group.execute(my_plan);
    }
    ROS_INFO("Planned");
    sleep(2.0);
    ros::shutdown();
}
