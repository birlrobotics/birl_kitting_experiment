#!/usr/bin/env python
import rospy
import moveit_commander
import sys
import numpy
from tf.transformations import (
    translation_matrix,
    quaternion_matrix,
)
import copy
import ipdb
from birl_kitting_experiment import hardcoded_data


if __name__ == '__main__':
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_to_home_py')
    robot = moveit_commander.RobotCommander()
    group = moveit_commander.MoveGroupCommander("right_arm")
    group.set_max_velocity_scaling_factor(0.3)
    group.set_max_acceleration_scaling_factor(0.3)


    group.set_goal_position_tolerance(0.00001)
    group.set_goal_orientation_tolerance(0.00001)

    group.set_start_state_to_current_state()
    group.set_pose_target(hardcoded_data.home_pose)
    plan = group.plan()
    group.execute(plan, wait=True)

