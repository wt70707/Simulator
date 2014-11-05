#include <ros.h>
#include <pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/listener.h>


class PointCloudCollisionWarningNode
{
    PointCloudCollisionWarningNode(void) : transform_listener_(0)
    {
        nh.param<std::string>("point_cloud_topic", point_cloud_topic_name, "stereo/point_cloud");
        point_cloud_sub_  = nh_.subscribe(point_cloud_topic_name, 1, &PointCloudCollisionWarningNode::pointCloudCallback, this);
        nh.param<double>("warn_radius", warn_radius_, 1.0);
    }
    
    protected:
        ros::NodeHandle nh_;
        tf::Listener transform_listener_;
        std::string point_cloud_topic_name;
        ros::Subscriber point_cloud_sub_;
        double warn_radius_;
        

        void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr point_cloud_msg)
        {      
            // Create a PCL point cloud object, in the frame of reference of the robot,
            // so that any point within warn_radius_ of the origin is a potential danger point
            pcl::PointCloud<pcl::PointXYZRGB> cloud;
            pcl::PointCloud<pcl::PointXYZRGB> cloud_baselink_frame;
            pcl::fromROSMsg(point_cloud_msg, cloud);
            pcl_ros::transformPointCloud ("base_link", 0, cloud, point_cloud_msg->header.frame_id, cloud_baselink_frame, tf_listener_);

            // Check for points within warn_radius_ of the origin
            for (size_t i = 0; i < cloud->points.size (); ++i)
            {
                if ( sqrt(cloud_baselink_frame->points[i].x*cloud_baselink_frame->points[i].x +
                          cloud_baselink_frame->points[i].y*cloud_baselink_frame->points[i].y +
                          cloud_baselink_frame->points[i].z*cloud_baselink_frame->points[i].z) < warn_radius_
                    )
                    
                    ROS_WARN("There is an obstacle at %f, %f, %f (%s frame)", 
                        cloud_baselink_frame->points[i].x, cloud_baselink_frame->points[i].y, cloud_baselink_frame->points[i].z,
                        "base_link" );
                    
                    // break;
            }
        }

};

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "point_cloud_safety");
    PointCloudCollisionWarningNode pccwn();
    
    ros::spin();
}

