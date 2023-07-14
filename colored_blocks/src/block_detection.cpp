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


std::vector<pcl::PointXYZRGBA> cluster_and_centroids(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& cloud);
pcl::PointXYZRGBA transform_point(std::string frame, pcl::PointXYZRGBA point);
void call_back(const sensor_msgs::PointCloud2ConstPtr& point_cloud);

ros::Publisher pub;

int main (int argc, char** argv)
{
    // Initialize ROS
    ros::init(argc, argv, "block_detection");
    ros::NodeHandle nh;

    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub = nh.subscribe("/filtered_point_cloud", 1, call_back);
    //pub = nh.advertise<std::vector<pcl::PointXYZRGBA>>("/filtered_point_cloud", 1);
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

    //find potential blocks
    //cluster and find centroids
    std::vector<pcl::PointXYZRGBA> potential_blocks;
    potential_blocks = cluster_and_centroids(pcl_cloud);


    //filter out bad potential blocks
    static tf::TransformBroadcaster br;
    // Create a TransformListener
    tf::TransformListener listener;
    int i = 0;
    double temp;
    for(auto& point : potential_blocks){
        
        temp = point.x;
        point.x = point.z;
        point.z = -1*point.y;
        point.y = -1*temp;

        tf::Transform transform;
        transform.setOrigin(tf::Vector3(point.x, point.y, point.z) );
        tf::Quaternion q;
        q.setRPY(0, 0, M_PI);
        transform.setRotation(q);
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "camera_link", "point"  + std::to_string(i)));
        i++;

        point = transform_point("base", point);
        ROS_INFO("%.3f, %.3f, %.3f\n", point.x, point.y, point.z);
    }
    std::cout << "\n\n================\n\n";
    
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

pcl::PointXYZRGBA transform_point(std::string frame, pcl::PointXYZRGBA point)
{
    // Create a TransformListener
    tf::TransformListener listener;
    ros::Duration tfstall(5);
    // Create a PointStamped message with the point in the source frame
    geometry_msgs::PointStamped transformed_point;
    transformed_point.header.frame_id = "camera_link";
    transformed_point.point.x = point.x;
    transformed_point.point.y = point.y;
    transformed_point.point.z = point.z;

    try
    {
        listener.waitForTransform(frame, "camera_link", ros::Time(0), tfstall);
        listener.transformPoint(frame, transformed_point, transformed_point);
    }
    catch (tf2::TransformException& ex)
    {
        ROS_ERROR("%s", ex.what());
    }

    point.x = transformed_point.point.x;
    point.y = transformed_point.point.y;
    point.z = transformed_point.point.z;
    return point;
}
