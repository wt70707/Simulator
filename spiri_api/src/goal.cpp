#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <iostream>
#include <vector>
#include <string>
int main(int argc, char **argv)
{
    ros::init(argc, argv, "move_group_interface_demo", ros::init_options::AnonymousName);
    // start a ROS spinning thread
    bool success_execute=0;
    ros::NodeHandle node_handle;

    ros::AsyncSpinner spinner(1);
    spinner.start();
    moveit::planning_interface::MoveGroup group("spiri");

    //group.setPlannerId("PRMkConfigDefault");
    group.setPlannerId("RRTConnectkConfigDefault");
    //group.setPoseTarget(target_pose);
    //std::cout<<group.getJointValueTarget();
    double a=1.0;
    std::vector<double> group_variable_values;
    moveit_msgs::RobotState start_state;
    //group.setStartState(start_state);
    geometry_msgs::Transform transform;
    transform.translation.x=-2.0;
    transform.translation.y=0.3;
    transform.translation.z=0.0;
    //transform.rotation.w=1.0;
    //float trans[] = {0,0,1};
    //string mystring;
    //string = "virtual_join";
    //std::vector<int>::iterator it;
    start_state.multi_dof_joint_state.joint_names.push_back("virtual_join");
    start_state.multi_dof_joint_state.transforms.push_back(transform);
    start_state.joint_state.header.frame_id="/nav";
    start_state.multi_dof_joint_state.header.frame_id="/nav";
    //robot_state::RobotState start_state;
    //start_state.
    //geometry_msgs::Pose start_pose2;
    //start_pose2.orientation.w=1.0;
    //start_pose2.position.z=1.0;
    // const robot_state::JointModelGroup *joint_model_group =start_state.getJointModelGroup(group.getName());
    //start_state.setFromIK(joint_model_group, start_pose2);
    //group.setStartState(start_state);
group.setStartStateToCurrentState();
    group.setWorkspace(-10.0,-10.0,0.0,10.0,10.0,10.0);
    //sleep(5.0);
    //std::cout<<start_state;
    //std::cout<<start_state.getRobotModel();
    //std::cout<<start_state;
    //std::cout<<group.getCurrentState();
    //std::string name="spiri";
    group.getCurrentState()->copyJointGroupPositions(group.getCurrentState()->getRobotModel()->getJointModelGroup(group.getName()), group_variable_values);
    //std::cout<<group.getName();
    //std::cout<<group_variable_values;
    group_variable_values[0] = -4.0;
    group_variable_values[2] = 1.0;
    group.setJointValueTarget(group_variable_values);
    group.setPlanningTime(60.0);
    group.setNumPlanningAttempts(5.0);
    moveit::planning_interface::MoveGroup::Plan my_plan;
    group.plan(my_plan);
    group.move();
    /*
    geometry_msgs::Pose target_pose;
    target_pose.position.x=0;
    target_pose.position.y=0;
    target_pose.position.z=2;
    target_pose.orientation.x=group_variable_values[3];
    target_pose.orientation.y=group_variable_values[4];
    target_pose.orientation.z=group_variable_values[5];
    target_pose.orientation.w=group_variable_values[6];
    std::vector<geometry_msgs::Pose> waypoints;
    waypoints.push_back(target_pose);
    moveit_msgs::RobotTrajectory trajectory;

    double fraction=group.computeCartesianPath(waypoints,0.01,0.0,trajectory);
    ROS_INFO("Visualizing plan 4 (cartesian path) (%.2f%% acheived)",
          fraction * 100.0);
    std::cout<<trajectory;
    //group.setJointValueTarget(target_pose);
    
    moveit::planning_interface::MoveGroup::Plan my_plan;
    bool success = group.plan(my_plan);

    //sleep(5.0);
    // std::cout<<success;

    if(success==1)
    {
        ROS_INFO("Going to execute");
        //success_execute=group.asyncExecute(my_plan);
        //std::cout<<success_execute;
    }
  */
    ROS_INFO("Planned");
    //sleep(2.0);
    
    ros::shutdown();
}
