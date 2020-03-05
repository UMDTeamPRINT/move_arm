# move_arm
moves arm

## Installation Instructions
Make sure you use ROS kinetic
First make sure you have a [catkin workspace](http://wiki.ros.org/catkin/Tutorials/create_a_workspace) set up.
Now install universal [robotics packages](https://github.com/ros-industrial/universal_robot/tree/kinetic-devel):
```
cd $HOME/catkin_ws/src

# retrieve the sources (replace '$ROS_DISTRO' with the ROS version you are using)
git clone -b $ROS_DISTRO-devel https://github.com/ros-industrial/universal_robot.git

cd $HOME/catkin_ws

# checking dependencies (again: replace '$ROS_DISTRO' with the ROS version you are using)
rosdep update
rosdep install --rosdistro $ROS_DISTRO --ignore-src --from-paths src

# building
catkin_make

# activate this workspace
source $HOME/catkin_ws/devel/setup.bash
```

To start simulation, in a new terminal connect to ros and run:
```roslaunch ur_e_gazebo ur3e.launch limited:=true```

To start motion planner, in a new terminal connect to ros and run:
```roslaunch ur3_e_moveit_config ur3_e_moveit_planning_execution.launch sim:=true```

If you would like to see the motion planning in rviz:
connect to ros and run:
```roslaunch ur3_e_moveit_config moveit_rviz.launch config:=true```

You now have a working simulation with motion planning! :D

## To run:
After the previous stuff is running, you can clone this package with:
```
cd ~/catkin_ws/src
git clone https://github.com/UMDTeamPRINT/move_arm.git
```
To install dependencies run:
```cd ~/catkin_ws/
rosdep update
rosdep install --rosdistro $ROS_DISTRO --ignore-src --from-paths src
```

Now to run it, use:
```rosrun move_arm test.py```
