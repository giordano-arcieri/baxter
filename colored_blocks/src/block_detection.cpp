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


std::vector<pcl::PointXYZRGBA> cluster_and_centroids(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& cloud);
void call_back(const sensor_msgs::PointCloud2ConstPtr& point_cloud);

int main (int argc, char** argv)
{
    // Initialize ROS
    ros::init(argc, argv, "block_detection");
    ros::NodeHandle nh;

    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub = nh.subscribe("/filtered_point_cloud", 1, call_back);

    ros::spin();
}

void call_back(const sensor_msgs::PointCloud2ConstPtr& point_cloud)
{
    // make necessary decleration
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pcl_cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
    ros::Rate loop_rate(1);

    //convert input ros pointcloud to pcl pointcloud
    pcl::fromROSMsg(*point_cloud, *pcl_cloud);

    //cluster and find centroids
    std::vector<pcl::PointXYZRGBA> blocks_pos;
    blocks_pos = cluster_and_centroids(pcl_cloud);

    //plot centroids / publish them in tf
    static tf::TransformBroadcaster br;
    int i = 0;
    for(const auto& block_pos : blocks_pos){
        tf::Transform transform;
        transform.setOrigin(tf::Vector3(block_pos.z, -1*block_pos.x, -1*block_pos.y) );
        tf::Quaternion q;
        q.setRPY(0, 0, M_PI);
        transform.setRotation(q);
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "camera_depth_frame", "point " + std::to_string(i)));
        i++;
    }
    loop_rate.sleep();

}

std::vector<pcl::PointXYZRGBA> cluster_and_centroids(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& point_cloud)
{
    // Creating KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr kdTree(new pcl::search::KdTree<pcl::PointXYZRGBA>);
    kdTree->setInputCloud(point_cloud);

    std::vector<pcl::PointIndices> clusterIndices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGBA> ec;
    ec.setClusterTolerance(0.03); // in meters
    ec.setMinClusterSize(27); // Adjust the minimum cluster size according to your requirements
    ec.setMaxClusterSize(60); // Adjust the maximum cluster size according to your requirements
    ec.setSearchMethod(kdTree);
    ec.setInputCloud(point_cloud);
    ec.extract(clusterIndices);

    std::vector<pcl::PointXYZRGBA> centroids;
    for (const auto& cluster : clusterIndices)
    {
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr clusterCloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
        for (const auto& index : cluster.indices)
            clusterCloud->push_back(point_cloud->points[index]);

        pcl::PointXYZRGBA centroid;
        pcl::computeCentroid(*clusterCloud, centroid);
        centroids.push_back(centroid);
    }

    return centroids;
}
