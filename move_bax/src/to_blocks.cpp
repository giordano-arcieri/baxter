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
    move_group.setMaxVelocityScalingFactor(0.2);   
    move_group.setMaxAccelerationScalingFactor(0.2);   

    // Sleeping object 
    ros::Duration tfstall(5);
    ros::Duration stall(2);

    // Define target poses queue
    std::queue<geometry_msgs::Pose> target_poses;
    double treshold = 0.08;


    //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!


    //subscribe to a thing that publishes all missplaced blocks

    //find closest misplaced blocks

    //go to closest misplaced blocks

    //check if closest misplaced blocks is there

    //pick up closest misplaced blocks

    //move to pos it has to go

    //release

    //loop


    //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!


    

    // Execute poses from the queue
    while (!target_poses.empty())
    {
        try
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
                
            }
        }
        catch (const std::exception& e)
        {
            ROS_ERROR_STREAM("Exception occurred: " << e.what());
            
        }
        stall.sleep();
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
  //quatornian depends on xyz
  pose.orientation.x = 0;
  pose.orientation.y = 1;
  pose.orientation.z = 0;
  pose.orientation.w = 0;
  return pose;
}

    
    
