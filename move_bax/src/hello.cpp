#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
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
    ros::Duration aspe(0.3);

    // Define target joints
    std::vector<double> right_nutural = {0, -0.55, 0, 0.75, 0, 1.26, 0};
    std::vector<double> start_hello = {0.75, -0.1, -3, 1.5, -1.6, 0, 0};
    std::vector<double> right_hello = {0.75, -0.1, -3, 1.5, -1.6, -0.6, 0};
    std::vector<double> left_hello = {0.75, -0.1, -3, 1.5, -1.6, 0.6, 0};

    // Define target joints queue
    std::queue<std::vector<double>> target_joints;

    // Add target joint to the queue 
    target_joints.push(right_nutural); //right nutural
    target_joints.push(start_hello); //start hello
    target_joints.push(right_hello); //right hello
    target_joints.push(left_hello); //left hello
    target_joints.push(right_hello); //right hello
    target_joints.push(left_hello); //left hello
    target_joints.push(right_nutural); //right nutural

    // Execute joint poses from the queue
    while (!target_joints.empty())
    {
        // Get the next joint pose from the queue
        std::vector<double> target_joint = target_joints.front();
        target_joints.pop();

        // Plan and execute the motion
        move_group.setJointValueTarget(target_joint);

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
        aspe.sleep();
    }
    
    spinner.stop();

    return 0;
}



// RIGHT NUTURAL

//   position: 
//     x: 0.646359
//     y: -0.841325
//     z: 0.0630911
//   orientation: 
//     x: 0.381826
//     y: 0.922561
//     z: -0.0216019
//     w: 0.0512096


// START HELLO


//   position: 
//     x: 0.46224
//     y: -0.380594
//     z: 1.25947
//   orientation: 
//     x: 0.0389374
//     y: -0.0608822
//     z: 0.686284
//     w: 0.723735



// HELLO LEFT


//   position: 
//     x: 0.45525
//     y: -0.587062
//     z: 1.16163
//   orientation: 
//     x: 0.240508
//     y: -0.270903
//     z: 0.644169
//     w: 0.673657



// HELLO RIGHT

//   position: 
//     x: 0.478335
//     y: -0.120675
//     z: 1.20451
//   orientation: 
//     x: -0.198718
//     y: 0.191173
//     z: 0.658173
//     w: 0.700551
