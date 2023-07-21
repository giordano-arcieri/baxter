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
#include <colored_blocks/blocks.h>
#include <colored_blocks/block.h>

std::vector<pcl::PointXYZRGBA> cluster_and_centroids(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& cloud);
pcl::PointXYZRGBA transform_point(std::string frame, pcl::PointXYZRGBA point);
void call_back(const sensor_msgs::PointCloud2ConstPtr& point_cloud);
const char color(pcl::PointXYZRGBA point);
const char quadrent(pcl::PointXYZRGBA point);

ros::Publisher pub;

int main (int argc, char** argv)
{
    // Initialize ROS
    ros::init(argc, argv, "block_detection");
    ros::NodeHandle nh;

    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub = nh.subscribe("/filtered_point_cloud", 1, call_back);
    pub = nh.advertise<colored_blocks::blocks>("/missplaced_blocks", 1);
    ros::spin();
    return 0;
}

void call_back(const sensor_msgs::PointCloud2ConstPtr& point_cloud)
{
    // make necessary decleration
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pcl_cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
    ros::Rate loop_rate(60);
    ros::Duration stall(3);
    std::vector<colored_blocks::block> missplaced_blocks; //this will be what get published so the robot knows what blocks it needs to fix


    //convert input ros pointcloud to pcl pointcloud
    pcl::fromROSMsg(*point_cloud, *pcl_cloud);

    //find potential blocks
    //cluster and find centroids
    std::vector<pcl::PointXYZRGBA> potential_blocks;
    potential_blocks = cluster_and_centroids(pcl_cloud);
    

    //filter out bad potential blocks


    double temp;
    colored_blocks::block temp_block;
    for(auto& point : potential_blocks)
    {
        //adjust each point to make it with respect to base
        temp = point.x;
        point.x = point.z;
        point.z = -1*point.y;
        point.y = -1*temp;
        point = transform_point("base", point);
        ROS_INFO("PB at (%3f, %3f, %3f) color:'%c' quadtrent:'%c'", point.x, point.y, point.z, color(point), quadrent(point));

        //only put good potential blocks and missplaced blocks in missplaced blocks
        if(!(point.x > 1.2 || point.x < 0.165 || point.y > 0.455 || point.y < -0.462 || point.z > -0.15 || point.z < -0.2))
        {
            //only put the blocks in if it is a missplaced block
            const char color_block = color(point);
            const char quadtrent_block_is_in = quadrent(point);
            if(color_block != quadtrent_block_is_in){
                temp_block.pose.x = point.x; temp_block.pose.y = point.y - 0.05; temp_block.pose.z = point.z;
                missplaced_blocks.push_back(temp_block);
                ROS_INFO("Found block");
            }else{
                ROS_INFO("Found block in right place");
            }
        }
        else
        {
            ROS_INFO("Found block out of range");
        }
        
    }
    std::cout << "==============" << std::endl;

    colored_blocks::blocks message;
    message.data = missplaced_blocks;


    // static tf::TransformBroadcaster br;
    // tf::Transform transform;
    // tf::Quaternion q;

    // int i = 0;
    // for(auto& block : message.data)
    // {
    //     transform.setOrigin(tf::Vector3(block.pose.x, block.pose.y, block.pose.z));
    //     q.setRPY(0, 0, 0);
    //     transform.setRotation(q);
    //     br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base", "point"  + std::to_string(i)));
    //     i++;
    // }
    // std::cout << "===========" << std::endl;



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

const char color(pcl::PointXYZRGBA point)
{
    char color;

    // Filter the color based on your criteria here
    // Only consider four colors: green, red, blue, and yellow
    if (point.g > point.r && point.g > point.b)
    {
        // If the green channel is the highest, color is green
        color = 'g';
    }
    else if (point.b > point.r && point.b > point.g)
    {
        // If the blue channel is the highest, color is blue
        color = 'b';
    }
    else if (point.r > point.g && point.r > point.b)
    {
        // If the red channel is the highest, color is red or yellow
        if(point.g > 210)
        {
            color = 'y';
        }
        else
        {
            color = 'r';
        }
        
    }


    return color;
}

const char quadrent(pcl::PointXYZRGBA point)
{
    if(point.y < 0){
        if(point.x > 0.63){
            return 'r';
        }
        return 'b';
    }
    if(point.x > 0.63){
        return 'y';
    }
    return 'g';
}

