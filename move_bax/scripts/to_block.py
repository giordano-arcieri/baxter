#! /usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg



def main():

  #init move it commander
  moveit_commander.roscpp_initialize(sys.argv)

  rospy.init_node('move_to_blocks', anonymous=True)

  robot = moveit_commander.RobotCommander()
  scene = moveit_commander.PlanningSceneInterface()    
  group = moveit_commander.MoveGroupCommander("right_arm")
  display_trajectory_publisher = rospy.Publisher('/planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)

  
  group_variable_values = group.get_current_pose()

  rospy.loginfo(group_variable_values)


  rospy.sleep(5)

  moveit_commander.roscpp_shutdown()

if __name__ == '__main__':
    main()