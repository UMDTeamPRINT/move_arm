#!/usr/bin/env python
import sys
import rospy
import moveit_commander
import geometry_msgs.msg
from std_msgs.msg import String

def add_floor(scene, robot):
    rospy.sleep(2)
    box_name="floor"
    box_pose=geometry_msgs.msg.PoseStamped()
    box_pose.header.frame_id = robot.get_planning_frame()
    box_pose.pose.orientation.w=0
    box_pose.pose.position.z=-0.01
    box_pose.pose.position.y=0
    box_pose.pose.position.x=0
    scene.add_box(box_name,box_pose,size=(5,5,.01))

def main():
  moveit_commander.roscpp_initialize(sys.argv)
  rospy.init_node('move_group_python_interface_tutorial', anonymous=True)

  robot = moveit_commander.RobotCommander()
  scene = moveit_commander.PlanningSceneInterface()

  # Add/update floor before running planning
  add_floor(scene,robot)

if __name__ == '__main__':
  main()
