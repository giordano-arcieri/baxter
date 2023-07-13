#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Pose.h>
#include <queue>


int main(int argc, char** argv)
{
    ros::init(argc, argv, "say_hello");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // Setup MoveIt
    moveit::planning_interface::MoveGroupInterface move_group("right_arm");
    move_group.setPlanningTime(10.0);
    move_group.allowReplanning(true);

    // Set maximum velocity and acceleration
    move_group.setMaxVelocityScalingFactor(0.3);   
    move_group.setMaxAccelerationScalingFactor(0.2);   

    // Sleeping object 
    ros::Duration aspe(5);


    
    spinner.stop();
    ros::shutdown();

    return 0;
}


