#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>

pcl::PointCloud<pcl::PointXYZRGBA>::Ptr removeNoise(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& cloud);
void call_back(const sensor_msgs::PointCloud2ConstPtr& point_cloud);

ros::Publisher pub;

int main (int argc, char** argv)
{
    // Initialize ROS
    ros::init(argc, argv, "filter_point_cloud");
    ros::NodeHandle nh;

    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub = nh.subscribe("/camera/depth_registered/points", 1, call_back);

    // Create a ROS publisher for the output point cloud
    pub = nh.advertise<sensor_msgs::PointCloud2>("/filtered_point_cloud", 1);

    // Spin
    ros::spin();
}

void call_back(const sensor_msgs::PointCloud2ConstPtr& point_cloud)
{
    // make necessary decleration
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pcl_cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
    sensor_msgs::PointCloud2 output_pc;
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGBA>);
    
    //convert input ros pointcloud to pcl pointcloud
    pcl::fromROSMsg(*point_cloud, *pcl_cloud);

    //filter the z-axes removing very far points and very close points (So that we only see the table and blocks)
    pcl::PassThrough<pcl::PointXYZRGBA> pass;
    pass.setInputCloud(pcl_cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(0.65, 1.2);
    //pass.setNegative (true);
    pass.filter(*cloud_filtered);

    cloud_filtered = removeNoise(cloud_filtered);

    //remove table 

    // Perform plane segmentation
    pcl::SACSegmentation<pcl::PointXYZRGBA> seg;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(1000); // Adjust the max iterations according to your requirements
    seg.setDistanceThreshold(0.02); // Adjust the distance threshold according to your requirements
    seg.setInputCloud(cloud_filtered);
    seg.segment(*inliers, *coefficients);


    // Extract the largest planar surface
    pcl::ExtractIndices<pcl::PointXYZRGBA> extract;
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr largestSurface(new pcl::PointCloud<pcl::PointXYZRGBA>);
    extract.setInputCloud(cloud_filtered);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(*largestSurface);

    // Remove the largest surface from the input cloud
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr filteredCloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
    extract.setNegative(true);
    extract.filter(*filteredCloud);


    //convert back to ros pointcloud msg and publish it 
    pcl::toROSMsg(*filteredCloud, output_pc);
    pub.publish(output_pc);
}

pcl::PointCloud<pcl::PointXYZRGBA>::Ptr removeNoise(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& cloud)
{
    // Create the filtering object
    pcl::VoxelGrid<pcl::PointXYZRGBA> voxelGridFilter;
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudFiltered(new pcl::PointCloud<pcl::PointXYZRGBA>);

    voxelGridFilter.setInputCloud(cloud);
    voxelGridFilter.setLeafSize(0.01f, 0.01f, 0.01f); // Adjust the leaf size according to your requirements
    voxelGridFilter.filter(*cloudFiltered);

    return cloudFiltered;
}

