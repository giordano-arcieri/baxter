# baxter

Baxter sorts colored blocks! The table is devided into 4 quadrants, a green, blue, orange, and yellow one. 
There are green, blue, orange, and yellow blocks all over the table. Baxter's job is to grab blocks that 
are located in the wrong quadrant, and bring them to the right quadrant, basically, orginize them.

The following is step by step instructions on how to set everything up:

This can be all done on the baxter robot. Or on a seperate divice that has set it's ROS_MASTER_URI to baxter.
Keep in mind some launch files can only be launch on baxter. Specifically enabling the arms and joint_trajectory_action_server

For this project a primeSense point cloud camera was used along with the openni driver.

Prerequesites: 
    while in catkin_ws/src install the following:
    openni driver:
        sudo apt-get install ros-${ROS_DISTRO}-openni-camera
    baxter_common which holds all the neccessary descriptions for baxter
        git clone https://github.com/RethinkRobotics/baxter_common.git

To launch the game you will first need to launch:
    enable robot:
        rosrun baxter_tools enable_robot.py -e
    launch joint:
        rosrun baxter_interface joint_trajectory_action_server.py
    launch move_group:
        roslaunch baxter_moveit_config move_group_remap.launch
    launch openni:
        roslaunch openni_launch openni.launch

You can launch all of these individually or you can use the following launch file to launch all of the above:
    roslaunch launch launch_dependencies.launch
    
Then you will need to launch the perception and manipulation code:
    launch perception code:
        roslaunch perception perception.launch
    launch manipulation code:
        roslaunch manipulation manipulation.launch

Once again, you can launch these individually or you can launch:
    roslaunch launch launch_baxter_sorting_blocks.launch

This is a demonstration of baxter sorting blocks:
    YYYYY

Another fun launch file wich makes baxter wave his right arm is:
    roslaunch manipulation say_hello.launch

This is a demonstration of baxter waving his right arm:
    YYYYY