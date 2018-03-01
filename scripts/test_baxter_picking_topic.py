#!/usr/bin/env python
import rospy
from ar_track_alvar_msgs.msg import AlvarMarkers
import moveit_commander
import sys
import numpy
from tf.transformations import (
    translation_matrix,
    quaternion_matrix,
)
import copy
import ipdb

hover_height = 0.10

if __name__ == '__main__':
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('test_baxter_picking_topic_py')
    msg = rospy.wait_for_message("baxter_available_picking_pose", AlvarMarkers)
    robot = moveit_commander.RobotCommander()
    group = moveit_commander.MoveGroupCommander("right_arm")
    group.set_max_velocity_scaling_factor(0.3)
    group.set_max_acceleration_scaling_factor(0.3)


    group.set_goal_position_tolerance(0.00001)
    group.set_goal_orientation_tolerance(0.00001)

    for marker in msg.markers:
        pos = marker.pose.pose.position
        ori = marker.pose.pose.orientation
        base_to_marker_mat = numpy.dot(translation_matrix((pos.x, pos.y, pos.z)), quaternion_matrix((ori.x, ori.y, ori.z, ori.w)))

        rospy.loginfo('Press Enter to move to pre-pick pose of marker %s'%marker.id)
        raw_input()
        if rospy.is_shutdown():
            break
        goal = copy.deepcopy(marker.pose.pose)
        goal.position.x -= hover_height*base_to_marker_mat[0, 2]
        goal.position.y -= hover_height*base_to_marker_mat[1, 2]
        goal.position.z -= hover_height*base_to_marker_mat[2, 2]
        group.set_start_state_to_current_state()
        group.set_pose_target(goal)
        plan = group.plan()
        group.execute(plan, wait=True)

        rospy.loginfo('Press Enter to move to pick pose of marker %s'%marker.id)
        raw_input()
        if rospy.is_shutdown():
            break
        goal = copy.deepcopy(marker.pose.pose)
        group.set_start_state_to_current_state()
        group.set_pose_target(goal)
        plan = group.plan()
        group.execute(plan, wait=True)

        rospy.loginfo('Press Enter to move to pre-pick pose of marker %s'%marker.id)
        raw_input()
        if rospy.is_shutdown():
            break
        goal = copy.deepcopy(marker.pose.pose)
        goal.position.x -= hover_height*base_to_marker_mat[0, 2]
        goal.position.y -= hover_height*base_to_marker_mat[1, 2]
        goal.position.z -= hover_height*base_to_marker_mat[2, 2]
        group.set_start_state_to_current_state()
        group.set_pose_target(goal)
        plan = group.plan()
        group.execute(plan, wait=True)

        rospy.loginfo('Press Enter to move to next marker')
        raw_input()
        if rospy.is_shutdown():
            break

    rospy.loginfo("Done iterating markers")

