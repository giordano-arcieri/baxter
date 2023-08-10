#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Pose.h>
#include <queue>
#include <cmath>
#include <string>
#include <std_msgs/String.h>
#include <perception/Block.h>
#include <perception/BlockList.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/Char.h>
#include "std_srvs/Empty.h"
#include <string>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

geometry_msgs::Pose createPose(double x, double y, double z);
void call_back(const perception::BlockList& blocks);
double distance_from_gripper(perception::Block block);
int move_to(geometry_msgs::Pose target_pose);
int move_to_neutral(void);
int pick_up(void);

moveit::planning_interface::MoveGroupInterface* pMove_group;
ros::ServiceClient release;
ros::ServiceClient grip;
ros::ServiceClient calibrate;


int main(int argc, char** argv)
{
    //Init ROS
    ros::init(argc, argv, "game_driver");
    ros::NodeHandle nh;
    
    ros::Duration stall(2);

    ros::AsyncSpinner spinner(2);
    spinner.start();

    //Gripper clients
    grip = nh.serviceClient<std_srvs::Empty>("/grip");
    release = nh.serviceClient<std_srvs::Empty>("/release");
    calibrate = nh.serviceClient<std_srvs::Empty>("/calibrate");


    //Setup MoveIt
    moveit::planning_interface::MoveGroupInterface move_group("right_arm"); 
    moveit::planning_interface::PlanningSceneInterface scene;
    move_group.setPlanningTime(10.0);
    move_group.allowReplanning(true);
    pMove_group = &move_group;

    //make sure the robot is ready to start by moving him to neutural, 
    //calibrating its grippers, releasing its grippers if they are open, and setting constraints.
    std_srvs::Empty msg;
    calibrate.call(msg);
    release.call(msg);
    move_to_neutral();

    //Set up joint and position restraints
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

    //Apply constraints
    moveit_msgs::Constraints constraints;
    constraints.joint_constraints.push_back(jc1);
    constraints.joint_constraints.push_back(jc2);
    move_group.setPathConstraints(constraints);

    //Create a CollisionObject message for the table
    moveit_msgs::CollisionObject table;
    table.header.frame_id = "base";
    table.id = "table";

    //Define the table's shape and pose
    shape_msgs::SolidPrimitive cylinder_primitive;
    cylinder_primitive.type = cylinder_primitive.CYLINDER;
    cylinder_primitive.dimensions.resize(2);
    cylinder_primitive.dimensions[0] = 0.8;  // height
    cylinder_primitive.dimensions[1] = 0.48;  // radius

    geometry_msgs::Pose table_pose;
    table_pose.orientation.w = 1.0;
    table_pose.position.x = 0.65;
    table_pose.position.y = 0;
    table_pose.position.z = -0.6;

    //Set the shape and pose of the table
    table.primitives.push_back(cylinder_primitive);
    table.primitive_poses.push_back(table_pose);
    
    //Add table to scene
    scene.applyCollisionObject(table);

    //subscribe to blocks. Publishes pose and color of all detected blocks
    ros::Subscriber sub = nh.subscribe("/blocks", 1, call_back);

    ros::waitForShutdown();

    return 0;
}

void call_back(const perception::BlockList& blocks)
{

    ros::Duration stall(2);
    ros::Rate loop_rate(60);
    std_srvs::Empty msg;
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    geometry_msgs::Pose target_pose;
    perception::Block closest_block;

    //adjust speed of right arm
    pMove_group->setMaxVelocityScalingFactor(0.3);   
    pMove_group->setMaxAccelerationScalingFactor(0.1); 
    
    //move arm to nutural if no blocks detected
    //Since this measn the camera cannot see anything or that there are actually not blocks
    //ROS_INFO("Got block locations. List is of size %ld", blocks.data.size());
    if(blocks.data.size() == 0){
        move_to_neutral();
        return;
    }

    ////Test code. This will publish a tf based on where each block is in format 
    //"B 'char rapresenting color' 'char rapresenting quadrant' 'number indicating what block'"
    // int i = 0;
    // for(auto& block : blocks.data)
    // {
    //     ROS_INFO("Block at (%.3f,%.3f,%.3f) of color: '%c' in quadrant: '%c'", 
    //         block.pose.x, block.pose.y, block.pose.z, block.color.data, block.quadrant.data);


    //     transform.setOrigin(tf::Vector3(block.pose.x, block.pose.y, block.pose.z));
    //     tf::Quaternion q;
    //     q.setRPY(0, 0, 0);
    //     transform.setRotation(q);
    //     std::string block_name = "B";
    //     block_name.push_back((i++)+48);
    //     block_name.push_back(' ');
    //     block_name.push_back(block.color.data);
    //     block_name.push_back(' ');
    //     block_name.push_back(block.quadrant.data);
    //     block_name.push_back(' ');
    //     br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base", block_name));
        

    // }
    // stall.sleep();
    // return;


    //Find closest misplaced blocks
    double block_distance, smallest_distance = 100;
    for(auto& block : blocks.data)
    {
        //Only go to pick up the blocks that are in the wrong quadrant
        block_distance = distance_from_gripper(block);//figure out distance based on block
        if(block_distance > 0.01 && block_distance < smallest_distance && block.color.data() != block.quadrant.data()){
            closest_block = block;
            smallest_distance = block_distance;
        }
        
    }


    std::string color = closest_block.color;
    std::string quadrant = closest_block.quadrant;


    if(closest_block.position.z == 0){ //If closest_block.pose.z == 0 that means there are no blocks that are out of place.
        ROS_INFO("All blocks are in the right place!");
        move_to_neutral();
        return;
    }

    
    if(color == "?"){ //This should never happen
        ROS_ERROR("color not determand");
        return;
    }
    if(quadrant == "?"){ //This should never happen
        ROS_ERROR("quadrant not determand");
        return;
    }


    ROS_INFO("Found block of '%s' color in the '%s' quadrent", 
        color, quadrant);
    
    
    //Go to closest misplaced blocks
    double block_height = -0.115;
    target_pose = createPose(closest_block.position.x, closest_block.position.y, block_height);

    ROS_INFO("Moving to pick up pose ...");
    if(move_to(target_pose)){
        ROS_ERROR("Movement failed!");
        return;
    }

    ROS_INFO("Picking up block ...");
    if(pick_up()){
        ROS_ERROR("Failed to pick up block!");
        return;
    }


    //Figure out what quadrent I need to go at
    ROS_INFO("Moving to release block in the '%s' quadrent ...", color);
    double x = 0.63, y = 0, height = -0.1;
    if(color == "yellow"){ //if color is yello move to yellow quadrant
        x = 0.75;
        y = 0.16;
    }
    if(color == "red"){ //move to red quadrant
        x = 0.75;
        y = -0.12;
    }
    if(color == "blue"){ //move to blue quadrant
        x = 0.48;
        y = -0.12;
    }
    if(color == "green"){ //move to green quadrant
        x = 0.48;
        y = 0.11;
    }
    target_pose = createPose(x, y, height);
    

    //Go to that quadrent
    if(move_to(target_pose)){
        ROS_ERROR("Movement failed!");
        return;
    }


    //call release service
    ROS_INFO("Releasing block ...");
    release.call(msg);

}


double distance_from_gripper(perception::Block block)
{
    tf::TransformListener tfListener;
    double distance_gripper_from_block = -8000.0; //caluclate this using distance fomula
    try
    {
        tf::StampedTransform transform;
        // Get the latest transform from source_frame to target_frame
        tfListener.waitForTransform("base", "right_gripper", ros::Time(0), ros::Duration(3.0));
        tfListener.lookupTransform("base", "right_gripper", ros::Time(0), transform);
        distance_gripper_from_block = sqrt(
            (block.position.x - transform.getOrigin().x())*(block.position.x - transform.getOrigin().x()) + 
            (block.position.y - transform.getOrigin().y())*(block.position.y - transform.getOrigin().y()));

    }
    catch (tf::TransformException &ex)
    {
        ROS_WARN("%s", ex.what());
    }

    return distance_gripper_from_block;
}

int move_to(geometry_msgs::Pose target_pose)
{
    ros::Duration stall(1);
    if(pMove_group == NULL){
        ROS_ERROR("pMove_group is NULL");
        return 1;
    }
    try
    {
        
        //Plan and execute the motion
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
            return 1;
            
        }
    }
    catch (const std::exception& e)
    {
        ROS_ERROR_STREAM("Exception occurred: " << e.what());
        return 1;
    }
    stall.sleep();
    return 0;
}

geometry_msgs::Pose createPose(double x, double y, double z)
{
    geometry_msgs::Pose pose;
    pose.position.x = x;
    pose.position.y = y;
    pose.position.z = z;

    //Quatornian depends on xyz. Since we cant reach the full table at a 90 degree angle
    if(x < 0.7)
    { 
        //90 degree with respect to table
        pose.orientation.x = 0;
        pose.orientation.y = 1;
        pose.orientation.z = 0;
        pose.orientation.w = 0;
    }
    else
    { 
        //45 degree with respect to table
        pose.orientation.x = 0;
        pose.orientation.y = 1;
        pose.orientation.z = 0;
        pose.orientation.w = 0;
    }

    return pose;
}


int move_to_neutral(void){

    ros::Duration stall(1);
    if(pMove_group == NULL){
        ROS_ERROR("pMove_group is NULL");
        return 1;
    }
    try
    {
        pMove_group->setMaxVelocityScalingFactor(0.3);   
        pMove_group->setMaxAccelerationScalingFactor(0.1); 
        std::vector<double> right_neutral = {0, -0.55, 0, 0.75, 0, 1.26, 0};
        pMove_group->setJointValueTarget(right_neutral);
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        bool success = (pMove_group->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        if (success)
        {
            pMove_group->execute(my_plan);
            ROS_INFO("Moved to neutral.");

        }
        else
        {
            ROS_ERROR("Planning to neutral failed!");
            return 1;
        }
    }
    catch (const std::exception& e)
    {
        ROS_ERROR_STREAM("Exception occurred: " << e.what());
        return 1;
    }
    stall.sleep();
    return 0;

}

int pick_up(void)
{
    ros::Duration stall(1);
    std_srvs::Empty msg;
    geometry_msgs::Pose target_pose;
    ROS_INFO("Picking up block ... ");
    pMove_group->setMaxVelocityScalingFactor(0.06);   
    pMove_group->setMaxAccelerationScalingFactor(0.02); 
    target_pose = pMove_group->getCurrentPose().pose;
    target_pose.position.z -= 0.06;
    if(move_to(target_pose)){
        ROS_ERROR("Movement failed!");
        return 1; 
    }
    stall.sleep();
    grip.call(msg);
    stall.sleep();
    pMove_group->setMaxVelocityScalingFactor(0.3);   
    pMove_group->setMaxAccelerationScalingFactor(0.1); 
    target_pose = pMove_group->getCurrentPose().pose;
    target_pose.position.z += 0.1;
    if(move_to(target_pose)){
        ROS_ERROR("Movement failed!");
        return 1;
    }
    stall.sleep();
    return 0;
}