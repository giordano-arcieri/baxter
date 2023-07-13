#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "robot_movement_node");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();


    moveit::planning_interface::MoveGroupInterface move_group("right_arm");





    spinner.stop();
    ros::shutdown();

    return 0;
}



    // header: 
    // seq: 0
    // stamp: 1689263376.544901981
    // frame_id: world
    // pose: 
    // position: 
    // x: 0.64701
    // y: -0.841195
    // z: 0.0642322
    // orientation: 
    // x: 0.381996
    // y: 0.922456
    // z: -0.021313
    // w: 0.0519508
