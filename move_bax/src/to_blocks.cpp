#include <ros/ros.h>


int main (int argc, char** argv)
{
    // Initialize ROS
    ros::init(argc, argv, "move_to_blocks");
    ros::NodeHandle nh;

    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub;

    ros::spin();
}