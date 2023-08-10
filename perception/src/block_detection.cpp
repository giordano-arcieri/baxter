#include <iostream>
#include <cmath>
#include <vector>
#include <std_msgs/String.h>
#include <string>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/centroid.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/kdtree/kdtree.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <perception/Block.h>
#include <perception/BlockList.h>

std::vector<pcl::PointXYZRGBA> cluster_and_centroids(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& cloud);
pcl::PointXYZRGBA transform_point(std::string frame, pcl::PointXYZRGBA point);
void call_back(const sensor_msgs::PointCloud2ConstPtr& point_cloud);
std::string  color(pcl::PointXYZRGBA point);
std::string  quadrant(pcl::PointXYZRGBA point);

ros::Publisher pub;

int main (int argc, char** argv)
{
    // Initialize ROS
    ros::init(argc, argv, "block_detection");
    ros::NodeHandle nh;

    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub = nh.subscribe("/block_pointcloud", 1, call_back);
    pub = nh.advertise<perception::BlockList>("/blocks", 1);
    ros::spin();
    return 0;
}

void call_back(const sensor_msgs::PointCloud2ConstPtr& point_cloud)
{
    // make necessary decleration
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pcl_cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
    ros::Rate loop_rate(60);
    ros::Duration stall(3);
    std::vector<perception::Block> blocks; //this will be what get published so the robot knows what blocks it needs to fix


    //convert input ros pointcloud to pcl pointcloud
    pcl::fromROSMsg(*point_cloud, *pcl_cloud);

    //find potential blocks
    //cluster and find centroids
    std::vector<pcl::PointXYZRGBA> potential_blocks;
    potential_blocks = cluster_and_centroids(pcl_cloud);
    

    //filter out bad potential blocks


    double temp;
    perception::Block temp_block;
    for(auto& point : potential_blocks)
    {
        //adjust each point to make it with respect to base
        temp = point.x;
        point.x = point.z;
        point.z = -1*point.y;
        point.y = -1*temp - 0.025;
        point = transform_point("base", point);

        //remove all potential blocks that are in a position where they couldnt be blocks
        if(!(point.x > 1 || point.x < 0.23 || point.y > 0.45 || point.y < -0.4 || point.z < -0.14 || point.z > -0.1))
        {
            //put block in blocks
            std::string color_of_block = color(point);
            std::string quadtrent_block_is_in = quadrant(point);
            temp_block.position.x = point.x; temp_block.position.y = point.y; temp_block.position.z = point.z;
            temp_block.color = color_of_block; temp_block.quadrant = quadtrent_block_is_in;
            blocks.push_back(temp_block);
            ROS_INFO("PB at (%.3f, %.3f, %.3f) color:'%s' quadtrent:'%s'", point.x, point.y, point.z, color_of_block.c_str(), quadtrent_block_is_in.c_str());


        }
        else
        {
            ROS_INFO("Found block out of range");
        }
        
    }
    std::cout << "==============" << std::endl;

    perception::BlockList message;
    message.data = blocks;


    pub.publish(message);
    
    stall.sleep();
    loop_rate.sleep();

}

std::vector<pcl::PointXYZRGBA> cluster_and_centroids(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& point_cloud)
{
    // Creating KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr kdTree(new pcl::search::KdTree<pcl::PointXYZRGBA>);
    kdTree->setInputCloud(point_cloud);

    std::vector<pcl::PointIndices> clusterIndices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGBA> ec;
    ec.setClusterTolerance(0.04); // in meters
    ec.setMinClusterSize(10); // Adjust the minimum cluster size according to your requirements
    ec.setMaxClusterSize(50); // Adjust the maximum cluster size according to your requirements
    ec.setSearchMethod(kdTree);
    ec.setInputCloud(point_cloud);
    ec.extract(clusterIndices);

    std::vector<pcl::PointXYZRGBA> centroids;
    
    for (const auto& cluster : clusterIndices)
    {
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr clusterCloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
        for (const auto& index : cluster.indices){
            clusterCloud->push_back(point_cloud->points[index]);
        }
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

std::string color(pcl::PointXYZRGBA point)
{
    std::string color;
    color = "?";

    // Filter the color based on your criteria here
    // Only consider four colors: green, red, blue, and yellow
    if (point.b > point.r && point.b > point.g)
    {
        // If the blue channel is the highest, color is blue
        color = "blue";
    }
    else if (point.r > point.g && point.r > point.b)
    {
        // If the red channel is the highest, color is red if green is less then 200 while yellow if green is more
        if(point.g > 170)
        {
            color = "yellow";
        }
        else
        {
            color = "red";
        }
    }
    else //green is heighest
    {
        // if there is a lot of red its yellow else it is green
        if(point.r > 170)
        {
            color = "yellow";
        }
        else
        {
            color = "green";
        }
        
    }


    return color;
}

std::string quadrant(pcl::PointXYZRGBA point)
{
    std::string  quadrant;
    quadrant = "?";

    if(point.y < 0){ //0 is the verticle half line of table
        if(point.x > 0.639){ //0.639 is the horizontal half line of table
            quadrant = "red";
        }else{
            quadrant = "blue";
        }
    }
    else if(point.x > 0.639){
        quadrant = "yellow";
    }
    else{
        quadrant = "green";

    }
    return quadrant;
}

