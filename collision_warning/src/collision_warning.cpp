#include <iostream>
#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include "pcl_ros/point_cloud.h"
#include <pcl_ros/transforms.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Point.h>
#include <tf/transform_listener.h>

class PointCloudCollisionWarningNode
{
    public:
        PointCloudCollisionWarningNode(void);
    
    protected:
        ros::NodeHandle nh_;
        tf::TransformListener tf_listener_;
        std::string point_cloud_topic_name;
        ros::Subscriber point_cloud_sub_;
        double warn_radius_;
        void pointCloudCallback(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud);
};

PointCloudCollisionWarningNode::PointCloudCollisionWarningNode(void)
{
    nh_.param<std::string>("point_cloud_topic", point_cloud_topic_name, "stereo/points2");
    point_cloud_sub_  = nh_.subscribe<  pcl::PointCloud<pcl::PointXYZRGB> >(point_cloud_topic_name, 1, &PointCloudCollisionWarningNode::pointCloudCallback, this);
    nh_.param<double>("warn_radius", warn_radius_, 1.0);
}


void PointCloudCollisionWarningNode::pointCloudCallback(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud)
{      
    // Create a PCL point cloud object, in the frame of reference of the robot,
    // so that any point within warn_radius_ of the origin is a potential danger point
    //pcl::PointCloud<pcl::PointXYZRGB> cloud_baselink_frame;
    //pcl_ros::transformPointCloud ("base_link", ros::Time::now(), cloud, point_cloud_msg->header.frame_id, cloud_baselink_frame, tf_listener_);
    double min_dist = -1;
    geometry_msgs::Point closest_point;

    // Check for points within warn_radius_ of the origin
    for (size_t i = 0; i < cloud->points.size (); ++i)
    {
        
        double dist = sqrt(cloud->points[i].x*cloud->points[i].x +
                            cloud->points[i].y*cloud->points[i].y +
                            cloud->points[i].z*cloud->points[i].z );
        if (min_dist < 0 || dist < min_dist)
        {
            min_dist = dist;
            closest_point.x = cloud->points[i].x;
            closest_point.y = cloud->points[i].y;
            closest_point.z = cloud->points[i].z;
        }
        if ( dist < warn_radius_ )
        {
            ROS_WARN("There is an obstacle at %f, %f, %f (%s frame)", 
                cloud->points[i].x, cloud->points[i].y, cloud->points[i].z,
                "base_link" );
            
            // break;
        }
        
        if (cloud->points[i].x == cloud->points[i].x) // true if not nan
            ROS_INFO("Point @(%f, %f, %f) dist=%f", cloud->points[i].x, cloud->points[i].y, cloud->points[i].z, dist);
    }
    //ROS_INFO("Closest point was at %f, %f, %f (%s frame)", closest_point.x, closest_point.y, closest_point.z, "base_link");
}
        
int main(int argc, char* argv[])
{
    ros::init(argc, argv, "point_cloud_safety");
    
    PointCloudCollisionWarningNode pccwn;
    ros::spin();
}

