#include <iostream>
#include <cmath>
#include <vector>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/centroid.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/kdtree/kdtree.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>


void call_back(const sensor_msgs::PointCloud2ConstPtr& point_cloud);

ros::Publisher pub;

int main (int argc, char** argv)
{
    // Initialize ROS
    ros::init(argc, argv, "block_detection");
    ros::NodeHandle nh;

    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub = nh.subscribe("/camera/depth_registered/points", 1, call_back);

    //subscribe to point cloud

    //publish x and y tresh hold for quadrent object with {char "g", x_tresh = 3, y_tresh = 5}
    ros::spin();
}

void call_back(const sensor_msgs::PointCloud2ConstPtr& point_cloud)
{
    // make necessary decleration
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pcl_cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
    ros::Rate loop_rate(60);
    ros::Duration stall(10);
    
    stall.sleep();
    //convert input ros pointcloud to pcl pointcloud
    pcl::fromROSMsg(*point_cloud, *pcl_cloud);


    loop_rate.sleep();

}
