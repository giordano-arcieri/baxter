#include <ros/ros.h>

int main (int argc, char** argv)
{
    // Initialize ROS
    ros::init(argc, argv, "filter_point_cloud");
    ros::NodeHandle nh;

    // Spin
    ros::spin();
}