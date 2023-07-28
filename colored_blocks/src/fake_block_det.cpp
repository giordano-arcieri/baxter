#include <iostream>
#include <cmath>
#include <vector>
#include <ros/ros.h>
#include <colored_blocks/blocks.h>
#include <colored_blocks/block.h>
#include <std_msgs/Char.h>


ros::Publisher pub;

struct Point{
    double x;
    double y;
    double z;
    char c;
    char q;
};

int main (int argc, char** argv)
{
    // Initialize ROS
    ros::init(argc, argv, "block_detection");
    ros::NodeHandle nh;

    pub = nh.advertise<colored_blocks::blocks>("/missplaced_blocks", 1);
    ros::Rate loop_rate(60);
    ros::Duration stall(3);
    std::vector<colored_blocks::block> missplaced_blocks; //this will be what get published so the robot knows what blocks it needs to fix

    //find potential blocks
    //cluster and find centroids
    std::vector<Point> potential_blocks;

    
    Point point;

    point.x = 0.797719; point.y = -0.304734; point.z = -0.181269; point.c = 'b'; point.q = 'r';
    potential_blocks.push_back(point);

    point.x = 0.726044; point.y = -0.063942; point.z = -0.182586; point.c = 'g'; point.q = 'r';
    potential_blocks.push_back(point);

    point.x = 0.895222; point.y = 0.122833; point.z = -0.180950; point.c = 'r'; point.q = 'y';
    potential_blocks.push_back(point);

    point.x = 0.558855; point.y = 0.213015; point.z = -0.185250; point.c = 'r'; point.q = 'g';
    potential_blocks.push_back(point);

    point.x = 0.319701; point.y = 0.160382; point.z = -0.186395; point.c = 'g'; point.q = 'g';
    potential_blocks.push_back(point);

    point.x = 0.497886; point.y = -0.286235; point.z = -0.190117; point.c = 'b'; point.q = 'b';
    potential_blocks.push_back(point);

    
    double temp;
    colored_blocks::block temp_block;
    while(ros::ok()){
        for(auto& point : potential_blocks)
        {

            ROS_INFO("PB at (%3f, %3f, %3f) color:'%c' quadtrent:'%c'", point.x, point.y, point.z, point.c, point.q);

            //only put good potential blocks and missplaced blocks in missplaced blocks
            if(!(point.x > 1.2 || point.x < 0.165 || point.y > 0.455 || point.y < -0.462 || point.z > -0.15 || point.z < -0.2))
            {
                //only put the blocks in if it is a missplaced block
                const char color_block = point.c;
                const char quadtrent_block_is_in = point.q;
                if(color_block != quadtrent_block_is_in){
                    temp_block.pose.x = point.x; temp_block.pose.y = point.y; temp_block.pose.z = point.z; 
                    temp_block.color.data = point.c;
                    missplaced_blocks.push_back(temp_block);
                    ROS_INFO("Found block");
                }else{
                    ROS_INFO("Found block in right place");
                }
            }
            else
            {
                ROS_INFO("Found block out of range");
            }
            
        }
        std::cout << "==============" << std::endl;
        
        colored_blocks::blocks message;
        message.data = missplaced_blocks;
        pub.publish(message);
        
        stall.sleep();
        loop_rate.sleep();


        ros::spinOnce();

    }

    
    return 0;
}

