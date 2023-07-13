#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Pose.h>
#include <queue>

geometry_msgs::Pose createPose(double x, double y, double z);


int main(int argc, char** argv)
{
    ros::init(argc, argv, "go_to_blocks");
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
    ros::Duration tfstall(5);
    ros::Duration aspe(7);

    // Define target poses queue
    std::queue<geometry_msgs::Pose> target_poses;
    double treshold = 0.25;

    double x = 0.74, y = -0.31, z = -0.16;


    // Create target poses and add them to the queue
    target_poses.push(createPose(x, y, z + treshold));

    // Execute poses from the queue
    while (!target_poses.empty())
    {
        // Get the next pose from the queue
        geometry_msgs::Pose target_pose = target_poses.front();
        target_poses.pop();

        // Plan and execute the motion
        move_group.setPoseTarget(target_pose);

        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

        if (success)
        {
            ROS_INFO("Planning successful. Moving the robot...");
            move_group.execute(my_plan);
            ROS_INFO("Movement Succesful.");
        }
        else
        {
            ROS_ERROR("Planning failed!");
            return 1;
        }
    }

    spinner.stop();
    ros::shutdown();

    return 0;
}

// Function to create a target pose based on provided parameters
geometry_msgs::Pose createPose(double x, double y, double z)
{
  geometry_msgs::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = z;
  pose.orientation.x = 0;
  pose.orientation.y = 1;
  pose.orientation.z = 0;
  pose.orientation.w = 0;
  return pose;
}

