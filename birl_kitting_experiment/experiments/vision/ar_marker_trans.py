#!/usr/bin/env python  
import rospy
import math
import tf


if __name__ == '__main__':
    rospy.init_node('convert2pick_pose582')
    br_mark0 = tf.TransformBroadcaster()
    br_mark8 = tf.TransformBroadcaster()
    br_mark4 = tf.TransformBroadcaster()
    br_mark11 = tf.TransformBroadcaster()
    br_mark13 = tf.TransformBroadcaster()
    while not rospy.is_shutdown():      
        br_mark0.sendTransform((0, 0, 0),
        tf.transformations.quaternion_from_euler(math.pi,0, -(math.pi/2) ),
        rospy.Time.now(),
        "ar_marker_0_pick_frame",
        "ar_marker_0")   
        
        br_mark8.sendTransform((0, 0, 0),
        tf.transformations.quaternion_from_euler(math.pi,0, -(math.pi/2) ),
        rospy.Time.now(),
        "ar_marker_8_pick_frame",
        "ar_marker_8")
        
        br_mark4.sendTransform((0, 0, 0),
        tf.transformations.quaternion_from_euler(math.pi,0, -(math.pi/2) ),
        rospy.Time.now(),
        "ar_marker_4_pick_frame",
        "ar_marker_4") 

        br_mark11.sendTransform((0, 0, 0),
        tf.transformations.quaternion_from_euler(math.pi,0, -(math.pi/2) ),
        rospy.Time.now(),
        "ar_marker_11_pick_frame",
        "ar_marker_11") 

        br_mark13.sendTransform((0, 0, 0),
        tf.transformations.quaternion_from_euler(math.pi,0, -(math.pi/2) ),
        rospy.Time.now(),
        "ar_marker_13_pick_frame",
        "ar_marker_13") 
