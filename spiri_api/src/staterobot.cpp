#include <spiri_api/staterobot.h>
#include <iostream>
#include <ctime>
#include <boost/timer.hpp>
#include <sstream>
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

cv::Mat Staterobot::get_left_image()

{
    ros::NodeHandle n;
    n.setCallbackQueue(&image_queue);
    image_transport::ImageTransport it(n);
    sub=it.subscribe("/stereo/left/image_raw", 1, &Staterobot::image_left_callback,this);



    image_queue.callAvailable(ros::WallDuration(1.0));
    return this->left_image;

}

void Staterobot::image_left_callback(const sensor_msgs::ImageConstPtr & msg)
{


    left_image=cv_bridge::toCvShare(msg,"bgr8")->image;

}
cv::Mat Staterobot::get_right_image()

{
    // @todo there is a better way to do this. Callback to a subscriber. No need to create a node

    // @todo the image callback needs to be called once or else it return an older image.
    ros::NodeHandle n;
    n.setCallbackQueue(&image_queue);
    image_transport::ImageTransport it(n);
    sub=it.subscribe("/stereo/right/image_raw", 1, &Staterobot::image_right_callback,this);



    image_queue.callAvailable(ros::WallDuration(1.0));
    return this->right_image;

}
void Staterobot::image_right_callback(const sensor_msgs::ImageConstPtr & msg)
{


    right_image=cv_bridge::toCvShare(msg,"bgr8")->image;

}
cv::Mat Staterobot::get_bottom_image()

{
    ros::NodeHandle n;
    n.setCallbackQueue(&image_queue);
    image_transport::ImageTransport it(n);
    sub=it.subscribe("/downward_cam/camera/image", 1, &Staterobot::image_bottom_callback,this);



    image_queue.callAvailable(ros::WallDuration(1.0));
    return this->bottom_image;

}
void Staterobot::image_bottom_callback(const sensor_msgs::ImageConstPtr & msg)
{


    bottom_image=cv_bridge::toCvShare(msg,"bgr8")->image;

}

void Staterobot::save_image(const std::string path="",const std::string camera="")
{
    //std::cout<<path;
    cv::Mat image;
    if(camera=="left")
    {

            image=get_left_image();
    }
    else if(camera=="right")
    {
        image=get_right_image();
    }
    else if(camera=="bottom")
    {
        image=get_bottom_image();
    }
    else
    {
        std::cout<<"Please pass a valid camera type";

    }


    //cv::Mat image=get_left_image();

    cv::imwrite(path,image);
}
bool Staterobot::wait_goal()
{
    bool param;
    ros::param::get("execution",param);
    return param;


}

void Staterobot::callback_goal(const action_controller::MultiDofFollowJointTrajectoryActionResultPtr& msg)
{
    ROS_INFO("here");
    //std::cout<<"here";
    status=true;

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
            success_execution=group.asyncExecute(my_plan);
    }
    else
    {
        ROS_INFO("Couldn't find a valid plan");
    }
    ROS_INFO("out of this loop");
    spinner.stop();
    return success_execution;

}


void Staterobot::send_vel(float x,float y,float z)
{

    ROS_INFO("testing");
    ros::NodeHandle nh;
    ros::Publisher vel_chatter = nh.advertise<geometry_msgs::Twist>("cmd_vel",1000);
    // ROS needs time to connect a subscriber. We can have advertise options but this an easy way to do it.
    ros::Rate poll_rate(100);

    while(vel_chatter.getNumSubscribers()==0)
    {
        poll_rate.sleep();
    }
    geometry_msgs::Twist vel;

    vel.linear.x=x;
    vel.linear.y=y;
    vel.linear.z=z;
    std::cout<<vel;
    ROS_INFO("Publishing velocity");
    vel_chatter.publish(vel);
    ros::spinOnce();



}

