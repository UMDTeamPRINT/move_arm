#!/usr/bin/env python
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import argparse
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
import tf

def all_close(goal, actual, tolerance):
  """
  Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
  @param: goal       A list of floats, a Pose or a PoseStamped
  @param: actual     A list of floats, a Pose or a PoseStamped
  @param: tolerance  A float
  @returns: bool
  """
  all_equal = True
  if type(goal) is list:
    for index in range(len(goal)):
      if abs(actual[index] - goal[index]) > tolerance:
        return False

  elif type(goal) is geometry_msgs.msg.PoseStamped:
    return all_close(goal.pose, actual.pose, tolerance)

  elif type(goal) is geometry_msgs.msg.Pose:
    return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

  return True

def add_box(scene, robot, timeout=4):
  rospy.sleep(2)

  box_name="floor"
  box_pose=geometry_msgs.msg.PoseStamped()
  box_pose.header.frame_id = robot.get_planning_frame()
  box_pose.pose.orientation.w=0
  box_pose.pose.position.z=-0.01
  box_pose.pose.position.y=0
  box_pose.pose.position.x=0
  scene.add_box(box_name,box_pose,size=(5,5,.01))
  
  rospy.sleep(2)

def go_to_joint_state(move_group):
  joint_goal = move_group.get_current_joint_values()
  joint_goal[0] = pi/2
  joint_goal[1] = -pi/4
  joint_goal[2] = 0
  joint_goal[3] = 0
  joint_goal[4] = 0
  joint_goal[5] = 0

  move_group.go(joint_goal, wait=True)
  move_group.stop()

def go_to_pose_goal(move_group, desired_cart_pos, desired_rot):
  pose_goal = geometry_msgs.msg.Pose()
  pose_goal.position.x= desired_cart_pos['x']
  pose_goal.position.y= desired_cart_pos['y']
  pose_goal.position.z= desired_cart_pos['z']

  pose_goal.orientation.w= desired_rot['rot_w']
  pose_goal.orientation.x= desired_rot['rot_x']
  pose_goal.orientation.y= desired_rot['rot_y']
  pose_goal.orientation.z= desired_rot['rot_z']

  move_group.set_pose_target(pose_goal)

  plan=move_group.go(wait=True)
  move_group.stop()
  move_group.clear_pose_targets()

def main(args):
  moveit_commander.roscpp_initialize(sys.argv)
  rospy.init_node('move_group_python_interface_tutorial', anonymous=True)

  robot = moveit_commander.RobotCommander()
  scene = moveit_commander.PlanningSceneInterface()

  add_box(scene,robot)

  group_name = "manipulator"
  move_group = moveit_commander.MoveGroupCommander(group_name)

  display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)
  #go_to_joint_state(move_group)
  desired_cart_pos = {'x':args.pos[0], 'y':args.pos[1], 'z':args.pos[2]}

  if hasattr(args, 'rot'):
    quat = tf.transformations.quaternion_from_euler(args.rot[0],args.rot[1],args.rot[2])
    desired_rot = {'rot_x':quat[0], 'rot_y':quat[1], 'rot_z':quat[2], 'rot_w':quat[3]}
    go_to_pose_goal(move_group, desired_cart_pos, desired_rot)
  else:
    quat = tf.transformations.quaternions_from_euler(0,0,0)
    desired_rot = {'rot_x':quat[0], 'rot_y':quat[1], 'rot_z':quat[2], 'rot_w':quat[3]}
    go_to_pose_goal(move_group, desired_cart_pos, desired_rot)

  #adds floor for planning. Needs to be run once on initialization

if __name__ == '__main__':
  parser = argparse.ArgumentParser(description="Train or Use the Network?")
  parser.add_argument('--pos', type=float, nargs=3)
  parser.add_argument('--rot', type=float, nargs=3)
  args = parser.parse_args()
  main(args)
