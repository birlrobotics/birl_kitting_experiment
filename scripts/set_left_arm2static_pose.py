#!/usr/bin/env python

import baxter_interface
import rospy

if __name__ == '__main__':
    rospy.init_node("set_left_arm_py")
    names = ['left_e0', 'left_e1', 'left_s0', 'left_s1', 'left_w0', 'left_w1', 'left_w2']
    joints = [-0.8022719520640714, 1.7184419776286348, 0.21399031991001521, 0.324436936637765, 2.5862916083748075, -0.5825292041994858, -0.3566505331833587]
    combined = zip(names, joints)
    command = dict(combined)
    left = baxter_interface.Limb('left')
    left.move_to_joint_positions(command)
    print "Done"




