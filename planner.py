#!/usr/bin/env python
import sys
import copy
import tf
import geometry_msgs.msg
import moveit_commander
import rospy
import moveit_msgs.msg

class Planner(object):
    """Planner"""
    def __init__(self):
        super(Planner, self).__init__()

        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('planner', anonymous=True)

        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()

        group_name = "manipulator"
        move_group = moveit_commander.MoveGroupCommander(group_name)

        display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                        moveit_msgs.msg.DisplayTrajectory,
                                                        queue_size=20)

        planning_frame = move_group.get_planning_frame()
        eef_link = move_group.get_end_effector_link()
        group_names = robot.get_group_names()

        self.robot = robot
        self.scene = scene
        self.move_group = move_group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names

    def add_floor(self, timeout=4):
        rospy.sleep(2)
        box_name="floor"
        box_pose=geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = self.robot.get_planning_frame()
        box_pose.pose.orientation.w=0
        box_pose.pose.position.z=-0.01
        box_pose.pose.position.y=0
        box_pose.pose.position.x=0
        self.scene.add_box(box_name,box_pose,size=(5,5,.01))

    def create_plan(self, plan_file):
        planf = open(plan_file, 'r')
        lines = planf.readlines()

        waypoints=[]

        for line in lines:
            coords = line.split(" ")
            pose = geometry_msgs.msg.Pose()
            fcoords = [float(i) for i in coords]

            pose.position.x = fcoords[0]
            pose.position.y = fcoords[1]
            pose.position.z = fcoords[2]

            quat = tf.transformations.quaternion_from_euler(fcoords[3], fcoords[4], fcoords[5])
            pose.orientation.x = quat[0]
            pose.orientation.y = quat[1]
            pose.orientation.z = quat[2]
            pose.orientation.w = quat[3]

            # print "waypoint: "+coords[0]+", "+coords[1]+", "+coords[2]

            waypoints.append(copy.deepcopy(pose))

        (plan, fraction) = self.move_group.compute_cartesian_path(waypoints,
                                                                  0.0001, 0)

        return plan, fraction

    def display_trajectory(self, plan):
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = self.robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        self.display_trajectory_publisher.publish(display_trajectory)

    def execute_plan(self, plan):
        self.move_group.execute(plan, wait=True)

def main():
    planner = Planner()

    planner.add_floor()

    plan_file = "listofcoords"

    plan, fraction = planner.create_plan(plan_file)

    print " Press enter to display plan"
    raw_input()

    planner.display_trajectory(plan)

    print " Press enter to run plan"
    raw_input()

    planner.execute_plan(plan)

if __name__ == '__main__':
    main()
