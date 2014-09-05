#! /usr/bin/env python

import roslib;
import rospy

import sys
import numpy as np

import actionlib
import jaco_msgs.msg
import std_msgs.msg
import geometry_msgs.msg

import goal_generators

def cartesian_pose_client(position, orientation):
    """Send a cartesian goal to the action server."""
    action_address = '/' + str(sys.argv[1]) + '_arm_driver/arm_pose/arm_pose'
    client = actionlib.SimpleActionClient(action_address, jaco_msgs.msg.ArmPoseAction)
    client.wait_for_server()

    goal = jaco_msgs.msg.ArmPoseGoal()
    goal.pose.header = std_msgs.msg.Header(frame_id=(str(sys.argv[1]) + '_api_origin'))
    goal.pose.pose.position = geometry_msgs.msg.Point(
        x=position[0], y=position[1], z=position[2])
    goal.pose.pose.orientation = geometry_msgs.msg.Quaternion(
        x=orientation[0], y=orientation[1], z=orientation[2], w=orientation[3])

    client.send_goal(goal)

    if client.wait_for_result(rospy.Duration(10.0)):
        return client.get_result()
    else:
        client.cancel_all_goals()
        print('        the cartesian action timed-out')
        return None

def gripper_client(finger_positions):
    """Send a gripper goal to the action server."""
    action_address = '/' + str(sys.argv[1]) + '_arm_driver/fingers/finger_positions'
    client = actionlib.SimpleActionClient(action_address,
                                          jaco_msgs.msg.SetFingersPositionAction)
    client.wait_for_server()

    goal = jaco_msgs.msg.SetFingersPositionGoal()
    goal.fingers.finger1 = float(finger_positions[0])
    goal.fingers.finger2 = float(finger_positions[1])

    # The MICO arm has only two fingers, but the same action definition is used
    if len(finger_positions) < 3:
        goal.fingers.finger3 = 0.0
    else:
        goal.fingers.finger3 = float(finger_positions[2])

    client.send_goal(goal)
    if client.wait_for_result(rospy.Duration(5.0)):
        return client.get_result()
    else:
        client.cancel_all_goals()
        print('        the gripper action timed-out')
        return None

def approach():
  pass
  
def grab():
  # Make sure fingers are opened
  
  # Move arm towards the "bucket"
  
  # Start grip
  
  # End grip
  pass
  
def position_arm():
  pass
  
def shower():
  pass

if __name__ == '__main__':
  rospy.init_node('ice_bucket_challenge')
  
  try:
    approach()
    rospy.sleep(1)
    grab()
    rospy.sleep(1)
    position_arm()
    rospy.sleep(1)
    shower()
  except rospy.ROSInterruptException:
    print "program interrupted before completion"