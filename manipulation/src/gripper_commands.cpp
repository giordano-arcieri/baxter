#include <ros/ros.h>
#include <baxter_core_msgs/EndEffectorCommand.h>
#include "std_srvs/Empty.h"

bool grip_command(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);
bool release_command(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);
bool calibrate_command(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);

ros::Publisher pub;


int main(int argc, char** argv)
{
    ros::init(argc, argv, "asd");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();
  
    pub = nh.advertise<baxter_core_msgs::EndEffectorCommand>("/robot/end_effector/right_gripper/command", 1);
    
    //Three simple services that are std_srvs::Empty that close, release, or calibrate baxters grippers
    ros::ServiceServer grip = nh.advertiseService("/grip", grip_command);
    ros::ServiceServer release = nh.advertiseService("/release", release_command);
    ros::ServiceServer calibrate = nh.advertiseService("/calibrate", calibrate_command);

    return 0;
}

bool grip_command(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
    baxter_core_msgs::EndEffectorCommand msg;
    ros::Duration stall(3);

    msg.command = "calibrate";
    msg.id = 65538;
    pub.publish(msg);

    stall.sleep();
}

bool release_command(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
    baxter_core_msgs::EndEffectorCommand msg;
    ros::Duration stall(3);

    msg.command = "calibrate";
    msg.id = 65538;
    pub.publish(msg);

    stall.sleep();
}

bool calibrate_command(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{   
    baxter_core_msgs::EndEffectorCommand msg;
    ros::Duration stall(3);

    msg.command = "calibrate";
    msg.id = 65538;
    pub.publish(msg);

    stall.sleep();
}