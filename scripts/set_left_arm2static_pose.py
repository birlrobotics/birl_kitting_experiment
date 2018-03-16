#!/usr/bin/env python

import baxter_interface
import rospy

if __name__ == '__main__':
    rospy.init_node("set_left_arm_py")
    names = ['left_e0', 'left_e1', 'left_s0', 'left_s1', 'left_w0', 'left_w1', 'left_w2']
    joints = [-0.9138690543827352, 1.5075196192943903, 0.28877188331942916, 0.6546263012301003, 2.3496750718434827, -0.47821851062327775, -0.02454369260616662]

    combined = zip(names, joints)
    command = dict(combined)
    left = baxter_interface.Limb('left')
    left.move_to_joint_positions(command)
    print "Done"




