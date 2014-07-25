#include "staterobot.h"
#include <iostream>
Staterobot::Staterobot()
{
    int dummy_argc=0;
    ros::init((int&)dummy_argc,NULL,"spiri_api");
    ros::NodeHandle n;
    success_plan=false;
    success_execution=false;

}

geometry_msgs::Quaternion Staterobot::get_imu()
{

    imu=ros::topic::waitForMessage<sensor_msgs::Imu>("/raw_imu");
    return imu->orientation;
}

geometry_msgs::Pose Staterobot::get_state()
{
    odom=ros::topic::waitForMessage<nav_msgs::Odometry>("/ground_truth/state");
    return odom->pose.pose;

}


float Staterobot::get_height_pressure()
{
    pressure=ros::topic::waitForMessage<geometry_msgs::PointStamped>("pressure_height");
    return pressure->point.z;
}

sensor_msgs::NavSatFixConstPtr Staterobot::get_gps_data()
{
    gps=ros::topic::waitForMessage<sensor_msgs::NavSatFix>("/fix");
    // @todo Need to return three floats or some sort of structure
    return gps;
}

geometry_msgs::Vector3 Staterobot::get_gps_vel()
{
    gps_vel=ros::topic::waitForMessage<geometry_msgs::Vector3Stamped>("/fix_velocity");
    return gps_vel->vector;
}

float Staterobot::get_height_altimeter()
{
    altimeter=ros::topic::waitForMessage<hector_uav_msgs::Altimeter>("/altimeter");
    return altimeter->altitude;
}

bool Staterobot::send_goal(float x,float y,float z, bool relative=false)
{
    ros::AsyncSpinner spinner(1);
    spinner.start();
    moveit::planning_interface::MoveGroup group("spiri");
    group.setPlannerId("PRMkConfigDefault");
    current_state=get_state();
    transform.translation.x=current_state.position.x;
    transform.translation.y=current_state.position.y;
    transform.translation.z=current_state.position.z;
    transform.rotation.x=current_state.orientation.x;
    transform.rotation.y=current_state.orientation.y;
    transform.rotation.z=current_state.orientation.z;
    transform.rotation.w=current_state.orientation.w;
    start_state.multi_dof_joint_state.joint_names.push_back("virtual_join");
    start_state.multi_dof_joint_state.transforms.push_back(transform);
    start_state.joint_state.header.frame_id="/nav";
    start_state.multi_dof_joint_state.header.frame_id="/nav";
    group.setStartState(start_state);
    // after setting the start state send the goal
    group.getCurrentState()->copyJointGroupPositions(group.getCurrentState()->getRobotModel()->getJointModelGroup(group.getName()), group_variable_values);
    if (relative==true)
    {

        group_variable_values[0]=current_state.position.x+x;
        group_variable_values[1]=current_state.position.y+y;
        group_variable_values[2]=current_state.position.z+z;
    }
    else
    {
        group_variable_values[0]=x;
        group_variable_values[1]=y;
        group_variable_values[2]=z;
    }
    group.setJointValueTarget(group_variable_values);

    moveit::planning_interface::MoveGroup::Plan my_plan;
    // @todo Can combine plan and execute into one by using move.
    success_plan = group.plan(my_plan);
    if(success_plan==1)
    {
            ROS_INFO("Going to execute");
            success_execution=group.execute(my_plan);
    }
    else
    {
        ROS_INFO("Couldn't find a valid plan");
    }
    spinner.stop();
    return success_execution;

}

int main(int argc, char **argv)
{
    //int dummy_argc=0;
    //ros::init((int&)dummy_argc,NULL,"listener");
    //ros::init(argc,argv,"listener");
    Staterobot robot;
    //robot.get_imu();
    //robot.get_height_altimeter();
    robot.send_goal(1.0,0.0,1.0);
    //ros::AsyncSpinner spinner(1);
    //spinner.start();
    //sleep(2.0);
    //std::cout<<robot.orientation;
    //return 0;

}


