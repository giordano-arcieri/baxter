#! /usr/bin/env python

from __future__ import print_function
from six.moves import input

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg


from math import pi, fabs, cos, sqrt

tau = 2.0 * pi

def dist(p, q):
    return sqrt(sum((p_i - q_i) ** 2.0 for p_i, q_i in zip(p, q)))


from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list


def main():

    #init move it commander
    moveit_commander.roscpp_initialize(sys.argv)

    rospy.init_node('say_hello', anonymous=True)

    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()    
    move_group = moveit_commander.MoveGroupCommander("right_arm")
    display_trajectory_publisher = rospy.Publisher('/planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=20)


    # Printing state of robot
    print("============ Printing robot state")
    print(robot.get_current_state())
    print("============ Done printing robot state")

    # Printing robot current values
    print("============ Printing robot current values")
    print(move_group.get_current_joint_values())
    print("============ Done printing robot current values")


    rospy.sleep(5)

    moveit_commander.roscpp_shutdown()

if __name__ == '__main__':
    main()