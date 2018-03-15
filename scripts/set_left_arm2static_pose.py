#!/usr/bin/env python

import baxter_interface
import rospy

if __name__ == '__main__':
    rospy.init_node("set_left_arm_py")
    names = ['left_e0', 'left_e1', 'left_s0', 'left_s1', 'left_w0', 'left_w1', 'left_w2']
    joints = [-0.4950922992900173, 2.0459468758421706, 0.4686311306989939, 0.13307283334905964, 1.7932235410380486, -0.7129175711697461, 0.49700977527487405]

    combined = zip(names, joints)
    command = dict(combined)
    left = baxter_interface.Limb('left')
    left.move_to_joint_positions(command)
    print "Done"




