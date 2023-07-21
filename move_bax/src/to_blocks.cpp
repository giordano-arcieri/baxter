#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Pose.h>
#include <queue>
#include <cmath>
#include <colored_blocks/block.h>
#include <colored_blocks/blocks.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/Char.h>
#include "std_srvs/Empty.h"


geometry_msgs::Pose createPose(double x, double y, double z);
void call_back(const colored_blocks::blocks& missplaced_blocks);
const char color(colored_blocks::block block);
const char quadrent(colored_blocks::block block);
double distance_from_gripper(colored_blocks::block block);
int move_to(geometry_msgs::Pose target_pose);

moveit::planning_interface::MoveGroupInterface* pMove_group;
ros::ServiceClient release;
ros::ServiceClient grip;



int main(int argc, char** argv)
{
    ros::init(argc, argv, "go_to_blocks");
    ros::NodeHandle nh;
    
    ros::AsyncSpinner spinner(2);
    spinner.start();

    grip = nh.serviceClient<std_srvs::Empty>("/grip");
    release = nh.serviceClient<std_srvs::Empty>("/release");


    // Setup MoveIt
    moveit::planning_interface::MoveGroupInterface move_group("right_arm"); 
    pMove_group = &move_group;

    moveit_msgs::JointConstraint jc1;
    moveit_msgs::JointConstraint jc2;
    jc1.joint_name = "right_e0";
    jc1.position = -0.2;
    jc1.tolerance_above = 1.6;
    jc1.tolerance_below = 1.3;

    jc2.joint_name = "right_w0";
    jc2.position = -0.1;
    jc2.tolerance_above = 2;
    jc2.tolerance_below = 1.6;

    // moveit_msgs::OrientationConstraint ocm;
    // ocm.link_name = "r_wrist_roll_link";
    // ocm.header.frame_id = "base_link";
    // ocm.orientation.w = 1.0;
    // ocm.absolute_x_axis_tolerance = 0.1;
    // ocm.absolute_y_axis_tolerance = 0.1;
    // ocm.absolute_z_axis_tolerance = 0.1;
    // ocm.weight = 1.0;
    // Now, set it as the path constraint for the group.

    // moveit_msgs::Constraints test_constraints;
    // test_constraints.orientation_constraints.push_back(ocm);
    // group.setPathConstraints(test_constraints);

    moveit_msgs::Constraints test_constraints;
    test_constraints.joint_constraints.push_back(jc1);
    test_constraints.joint_constraints.push_back(jc2);
    move_group.setPathConstraints(test_constraints);

    move_group.setPlanningTime(10.0);
    move_group.allowReplanning(true);
    
    // Set maximum velocity and acceleration
    move_group.setMaxVelocityScalingFactor(0.2);   
    move_group.setMaxAccelerationScalingFactor(0.1);  

    //subscribe to a thing that publishes all missplaced blocks
    ros::Subscriber sub = nh.subscribe("/missplaced_blocks", 1, call_back);

    ros::waitForShutdown();

    return 0;
}

void call_back(const colored_blocks::blocks& missplaced_blocks)
{


    // Sleeping object 
    ros::Duration tfstall(2);
    ros::Duration stall(1);
    ros::Rate loop_rate(60);
    std_srvs::Empty msg;



    release.call(msg);
    pMove_group->setMaxVelocityScalingFactor(0.3);   
    pMove_group->setMaxAccelerationScalingFactor(0.1); 
    
    //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    ROS_INFO("Got block locations. List is of size %d", missplaced_blocks.data.size());

    //find closest misplaced blocks
    colored_blocks::block closest_block;
    double block_distance, smallest_distance = 100;
    for(auto& block : missplaced_blocks.data)
    {
        block_distance = distance_from_gripper(block);//figure out distance based on block
        if(block_distance > 0.01 && block_distance < smallest_distance){
            closest_block = block;
            smallest_distance = block_distance;
        }
    }
    char c = closest_block.color.data;
    if(c == '?'){
        ROS_ERROR("color not determand");
        return;
    }
    ROS_INFO("Found closest block color:'%c' quadrent:'%c'", 
        c, quadrent(closest_block));
    stall.sleep();
    //go to closest misplaced blocks
    // Define target poses queue
    double height_above_block = 0.09;
    geometry_msgs::Pose target_pose = createPose(closest_block.pose.x, closest_block.pose.y, closest_block.pose.z + height_above_block);

    ROS_INFO("Moving to pick up pose ...");
    stall.sleep();
    if(move_to(target_pose)){
        return;
    }
 
    stall.sleep();
    


    //call chris pick up server
    pMove_group->setMaxVelocityScalingFactor(0.03);   
    pMove_group->setMaxAccelerationScalingFactor(0.01); 
    stall.sleep();
    ROS_INFO("Picking up block ...");
    target_pose = pMove_group->getCurrentPose().pose;
    target_pose.position.z -= 0.07;
    if(move_to(target_pose)){
        return;
    }
    stall.sleep();
    stall.sleep();
    grip.call(msg);
    stall.sleep();
    target_pose = pMove_group->getCurrentPose().pose;
    target_pose.position.z += 0.1;
    if(move_to(target_pose)){
        return;
    }
    stall.sleep();
    pMove_group->setMaxVelocityScalingFactor(0.2);   
    pMove_group->setMaxAccelerationScalingFactor(0.1); 

    //figure out what quadrent I need to go at
    ROS_INFO("Moving to release pose quadrent = '%c' ...", c);
    double x = 0.63, y = 0, height = -0.1;
    if(c == 'y'){
        x = 0.75;
        y = 0.16;
    }
    if(c == 'r'){
        x = 0.75;
        y = -0.12;
    }
    if(c == 'b'){
        x = 0.48;
        y = -0.12;
    }
    if(c == 'g'){
        x = 0.48;
        y = 0.11;
    }
    target_pose = createPose(x, y, height);
    
    //go to that quadrent
    if(move_to(target_pose)){
        return;
    }

    stall.sleep();
 

    //call chris release server
    ROS_INFO("Releasing block ...");
    release.call(msg);


    //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
}


const char quadrent(colored_blocks::block block)
{
    geometry_msgs::Point point = block.pose;
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

double distance_from_gripper(colored_blocks::block block){
    tf::TransformListener tfListener;
    double distance_gripper_from_block = 1000.0; //caluclate this using distance fomula
    try
    {
        tf::StampedTransform transform;
        // Get the latest transform from source_frame to target_frame
        tfListener.waitForTransform("base", "right_gripper", ros::Time(0), ros::Duration(3.0));
        tfListener.lookupTransform("base", "right_gripper", ros::Time(0), transform);
        distance_gripper_from_block = sqrt(
            (block.pose.x - transform.getOrigin().x())*(block.pose.x - transform.getOrigin().x()) + 
            (block.pose.y - transform.getOrigin().y())*(block.pose.y - transform.getOrigin().y()));

    }
    catch (tf::TransformException &ex)
    {
        ROS_WARN("%s", ex.what());
    }

    return distance_gripper_from_block;
}

int move_to(geometry_msgs::Pose target_pose)
{
    if(pMove_group == NULL){
        ROS_ERROR("pMove_group is NULL");
        return 1;
    }
    try
    {
        
        // Plan and execute the motion
        pMove_group->setPoseTarget(target_pose);

        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        bool success = (pMove_group->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

        if (success)
        {
            ROS_INFO("Planning successful. Moving the robot...");
            pMove_group->execute(my_plan);
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
        return 1;
    }
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