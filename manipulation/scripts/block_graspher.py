#! /usr/bin/env python3
import rospy
import moveit_commander
import sys
from argparse import ArgumentParser
import json
import os
import tf
import tf.transformations as TFMath
import math
import tf
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import EmptyResponse, Empty
from baxter_core_msgs.msg import EndEffectorCommand
from perception.msg import BlockList

FILE_DIR = os.path.dirname(__file__)

Vector3 = (float, float, float)

class BlockGrasper():
    def __init__(self, limb="right", *, namespace = None):
        moveit_commander.roscpp_initialize(sys.argv)
        self.limb = limb
        self.move_group = moveit_commander.MoveGroupCommander(arguments.limb + "_arm")
        self.tf_listener = tf.TransformListener(True, rospy.Duration(5))
        self.move_group.allow_replanning(True)
        self.closest_block = None
        self.command = EndEffectorCommand()
        self.command.id = 65538
        self.publisher = rospy.Publisher(f"/robot/end_effector/{limb}_gripper/command", EndEffectorCommand)
        rospy.Subscriber(f"blocks/{namespace}/color", BlockList, self.closest_block)

    def closest_block(self, block_list):
        closest_block = None 
        closest_dist = None
        for block in block_list.blocks:
            if closest_dist is None or closest_dist > block.distance:
                closest_block = block
                closest_dist = block.distance
        self.closest_block = closest_block
        
    def move_to_closest_block(self, _):
        QUATERNION_ROTATED = TFMath.quaternion_from_euler(-math.pi, -math.pi/8, 0.0)
        QUATERNION_VERTICLE = TFMath.quaternion_from_euler(-math.pi, 0.0000, 0.00000)

        self.move_group.set_max_acceleration_scaling_factor(0.02)
        self.move_group.set_max_velocity_scaling_factor(1.0)

        self.block_locater.wait_for_camera_tf()
    
        rate = rospy.Rate(10)

        block = None
        while not block:
            rospy.loginfo("[[No blocks found]]")
            block = self.block_locater.locate_closest_block()    
            rate.sleep()

        position = (
            self.closest_block.position.x,
            self.closest_block.position.x,
            self.closest_block.position.x + 0.040,
        )

        pose = convert_vector3_to_pose(position, QUATERNION_VERTICLE, target="/base")

        self.move_group.set_pose_target(pose)
        self.move_group.go(wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()

        return EmptyResponse()
    
    def pick_up(self, _):
        rospy.loginfo("pick_up")
        self.command.command='grip'
        self.publisher.publish(self.command)
        rospy.sleep(rospy.Duration(1))
        return EmptyResponse()
    
    def place(self, _):
        rospy.loginfo("place")
        self.command.command ='release'
        self.publisher.publish(self.command)
        rospy.sleep(rospy.Duration(1))
        return EmptyResponse()

    def calibrate(self, _):
        rospy.loginfo("calibrate")
        self.command.command ='calibrate'
        self.publisher.publish(self.command)
        rospy.sleep(rospy.Duration(1))
        return EmptyResponse()

def convert_vector3_to_pose(point: Vector3, quaternion: tuple, *, target: Optional[str]):
    pose = PoseStamped()
    pose.header.frame_id = target
    pose.pose.orientation.x = quaternion[0]
    pose.pose.orientation.y = quaternion[1]
    pose.pose.orientation.z = quaternion[2]
    pose.pose.orientation.w = quaternion[3]
    pose.pose.position.x = point[0]
    pose.pose.position.y = point[1]
    pose.pose.position.z = point[2]
    return pose

if __name__ == "__main__":
    rospy.init_node("gripper_controller")

    parser = ArgumentParser()
    parser.add_argument("--config", "-c")
    parser.add_argument("--limb", "-l", default="left")

    arguments = parser.parse_args()

    with open(os.path.join(FILE_DIR, arguments.config), "r") as file:
        config = json.loads(file.read())["gripper_config"]

    block_graspher = BlockGrasper(arguments.limb, config=config)

    for method_name in dir(block_graspher): #create services for all class methods
        attribute = getattr(block_graspher, method_name)
        if callable(attribute) and method_name[0] != "_":
            rospy.loginfo(attribute)
            rospy.Service(f"{method_name}", Empty, attribute)
    
    rospy.spin()