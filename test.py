#!/usr/bin/env python
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

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
  box_pose.pose.position.z=0
  box_pose.pose.position.y=0
  box_pose.pose.position.x=0
  scene.add_box(box_name,box_pose,size=(5,5,.01))

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

def go_to_pose_goal(move_group):
  pose_goal = geometry_msgs.msg.Pose()
  pose_goal.position.x=0
  pose_goal.position.y=0.4
  pose_goal.position.z=.5
  pose_goal.orientation.w=0

  move_group.set_pose_target(pose_goal)

  plan=move_group.go(wait=True)
  move_group.stop()
  move_group.clear_pose_targets()

def main():
  moveit_commander.roscpp_initialize(sys.argv)
  rospy.init_node('move_group_python_interface_tutorial', anonymous=True)

  robot = moveit_commander.RobotCommander()
  scene = moveit_commander.PlanningSceneInterface()

  group_name = "manipulator"
  move_group = moveit_commander.MoveGroupCommander(group_name)

  display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)
  #go_to_joint_state(move_group)
  go_to_pose_goal(move_group)
  #add_box(scene,robot)

if __name__ == '__main__':
  main()