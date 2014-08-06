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

std::vector<double> Staterobot::get_imu()
{

    imu=ros::topic::waitForMessage<sensor_msgs::Imu>("/raw_imu");
    std::vector<double> v(4);
    v[0]=imu->orientation.x;
    v[1]=imu->orientation.y;
    v[2]=imu->orientation.z;
    v[3]=imu->orientation.w;

    return v;
}

std::vector<double> Staterobot::get_state()
{
    odom=ros::topic::waitForMessage<nav_msgs::Odometry>("/ground_truth/state");
    std::vector<double> v(7);
    v[0]=odom->pose.pose.position.x;
    v[1]=odom->pose.pose.position.y;
    v[2]=odom->pose.pose.position.z;
    v[3]=odom->pose.pose.orientation.x;
    v[4]=odom->pose.pose.orientation.y;
    v[5]=odom->pose.pose.orientation.z;
    v[6]=odom->pose.pose.orientation.w;

    return v;

}


float Staterobot::get_height_pressure()
{
    pressure=ros::topic::waitForMessage<geometry_msgs::PointStamped>("pressure_height");
    return pressure->point.z;
}

std::vector<double> Staterobot::get_gps_data()
{
    gps=ros::topic::waitForMessage<sensor_msgs::NavSatFix>("/fix");
    std::vector<double> v(3);
    v[0]=gps->latitude;
    v[1]=gps->longitude;
    v[2]=gps->altitude;
    // @todo Need to return three floats or some sort of structure
    return v;
}

std::vector<double> Staterobot::get_gps_vel()
{
    gps_vel=ros::topic::waitForMessage<geometry_msgs::Vector3Stamped>("/fix_velocity");
    std::vector<double> v(3);
    v[0]=gps_vel->vector.x;
    v[1]=gps_vel->vector.y;
    v[2]=gps_vel->vector.z;

    return v;
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
    transform.translation.x=current_state[0];
    transform.translation.y=current_state[1];
    transform.translation.z=current_state[2];
    transform.rotation.x=current_state[3];
    transform.rotation.y=current_state[4];
    transform.rotation.z=current_state[5];
    transform.rotation.w=current_state[6];
    start_state.multi_dof_joint_state.joint_names.push_back("virtual_join");
    start_state.multi_dof_joint_state.transforms.push_back(transform);
    start_state.joint_state.header.frame_id="/nav";
    start_state.multi_dof_joint_state.header.frame_id="/nav";
    group.setStartState(start_state);
    // after setting the start state send the goal
    group.getCurrentState()->copyJointGroupPositions(group.getCurrentState()->getRobotModel()->getJointModelGroup(group.getName()), group_variable_values);
    if (relative==true)
    {

        group_variable_values[0]=current_state[0]+x;
        group_variable_values[1]=current_state[1]+y;
        group_variable_values[2]=current_state[2]+z;
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
    bool latch=1;
    ros::Publisher vel_chatter = nh.advertise<geometry_msgs::Twist>("cmd_vel",1,latch);


    geometry_msgs::Twist vel;

    vel.linear.x=x;
    vel.linear.y=y;
    vel.linear.z=z;
    
    vel_chatter.publish(vel);
    // this is hack but it looks like it is not possible in ROS
    sleep(1.0);
    //ros::spinOnce();



}

