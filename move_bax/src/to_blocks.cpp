#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Pose.h>
#include <queue>
#include <colored_blocks/block.h>
#include <colored_blocks/blocks.h>
#include <tf/transform_broadcaster.h>

geometry_msgs::Pose createPose(double x, double y, double z);
void call_back(const colored_blocks::blocks& missplaced_blocks);
const char color(colored_blocks::block block);

int main(int argc, char** argv)
{
    ros::init(argc, argv, "go_to_blocks");
    ros::NodeHandle nh;
    

    //subscribe to a thing that publishes all missplaced blocks
    ros::Subscriber sub = nh.subscribe("/missplaced_blocks", 1, call_back);

    // // Setup MoveIt
    // moveit::planning_interface::MoveGroupInterface move_group("right_arm");
    // move_group.setPlanningTime(10.0);
    // move_group.allowReplanning(true);

    // // Set maximum velocity and acceleration
    // move_group.setMaxVelocityScalingFactor(0.2);   
    // move_group.setMaxAccelerationScalingFactor(0.2); 

    ros::spin();
    ros::shutdown();

    return 0;
}

void call_back(const colored_blocks::blocks& missplaced_blocks)
{


    // Sleeping object 
    ros::Duration tfstall(5);
    ros::Duration stall(2);
    ros::Rate loop_rate(60);

    // Define target poses queue
    geometry_msgs::Pose target_pose;
    double height_above_block = 0.08;

    static tf::TransformBroadcaster br;
    tf::Transform transform;
    tf::Quaternion q;

    int i = 0;
    for(auto& block : missplaced_blocks.data)
    {
        transform.setOrigin(tf::Vector3(block.pose.x, block.pose.y, block.pose.z));
        q.setRPY(0, 0, 0);
        transform.setRotation(q);
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base", "point"  + std::to_string(i)));
        i++;
        ROS_INFO("Block of color '%c' at xz position %5f, %5f", color(block), block.pose.x, block.pose.z);
    }
    std::cout << "===========" << std::endl;



        






    //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!


    //find closest misplaced blocks

    //go to closest misplaced blocks

    //check if closest misplaced blocks is there

    //pick up closest misplaced blocks

    //move to pos it has to go

    //release

    //loop


    //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!


    


    // try
    // {

    //     // Plan and execute the motion
    //     move_group.setPoseTarget(target_pose);

    //     moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    //     bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    //     if (success)
    //     {
    //         ROS_INFO("Planning successful. Moving the robot...");
    //         //move_group.execute(my_plan);
    //         ROS_INFO("Movement Succesful.");
    //     }
    //     else
    //     {
    //         ROS_ERROR("Planning failed!");
            
    //     }
    // }
    // catch (const std::exception& e)
    // {
    //     ROS_ERROR_STREAM("Exception occurred: " << e.what());
        
    // }
    // stall.sleep();
    

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

    
    
const char color(colored_blocks::block block)
{
    char color;

    // Filter the color based on your criteria here
    // Only consider four colors: green, red, blue, and yellow
    if (block.color.g > block.color.r && block.color.g > block.color.b)
    {
        // If the green channel is the highest, color is green
        color = 'g';
    }
    else if (block.color.b > block.color.r && block.color.b > block.color.g)
    {
        // If the blue channel is the highest, color is blue
        color = 'b';
    }
    else if (block.color.r > block.color.g && block.color.r > block.color.b)
    {
        // If the red channel is the highest, color is red or yellow
        if (block.color.g > 210)
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