#!/usr/bin/env python
import rospy
from ar_track_alvar_msgs.msg import AlvarMarkers
import tf
from tf.transformations import (
    translation_matrix,
    quaternion_matrix,
    translation_from_matrix,
    quaternion_from_matrix,
)
import threading
import copy
import numpy
import ipdb

handcoded_marker_compensation = {
    0: numpy.array((
        (0.99995001, 0.00609497, 0.00792666, 0.017),
        (-0.005903, 0.99969405, -0.02401994, 0.017),
        (-0.00807064, 0.02397195, 0.99968005, 0.056),
        (0.0, 0.0, 0.0, 1.0)
    ), dtype=numpy.float64),
    8: numpy.array((
        (1.0, 0.0, 0.0, 0.0),
        (0.0, 1.0, 0.0, 0.0),
        (0.0, 0.0, 1.0, 0.0),
        (0.0, 0.0, 0.0, 1.0)
    ), dtype=numpy.float64),
    13: numpy.array((
        (1.0, 0.0, 0.0, 0.0),
        (0.0, 1.0, 0.0, 0.0),
        (0.0, 0.0, 1.0, 0.0),
        (0.0, 0.0, 0.0, 1.0)
    ), dtype=numpy.float64),
}

writable = threading.Event()
writable.clear()
shared_msg = None

def cb(msg):
    global writable, shared_msg
    if writable.is_set():
        shared_msg = msg

def transform_into_baxter_picking_space(mat):
    for axis in range(3):
        swap_with =  abs(mat[:3, axis]).argmax()
        if swap_with != axis:
            tmp = mat[:3, swap_with].copy()
            mat[:3, swap_with] = mat[:3, axis]
            mat[:3, axis] = tmp
    
    if mat[0][0] < 0:
        mat[:3, 0] = -mat[:3, 0]
    if mat[1][1] > 0:
        mat[:3, 1] = -mat[:3, 1]
    if mat[2][2] > 0:
        mat[:3, 2] = -mat[:3, 2]

    return mat
        

if __name__ == '__main__':
    rospy.init_node("alvar_marker_to_baxter_picking_pose_py")
    
    rospy.Subscriber("ar_pose_marker", AlvarMarkers, cb)

    writable.set()

    listener = tf.TransformListener()

    look_up_t = rospy.Time(0)
    listener.waitForTransform('base', 'camera_link', look_up_t, rospy.Duration(3))
    base_to_cam = listener.lookupTransform('base', 'camera_link', look_up_t)
    base_to_cam_mat = listener.fromTranslationRotation(*base_to_cam) 

    broadcaster = tf.TransformBroadcaster()
    pub = rospy.Publisher("baxter_available_picking_pose", AlvarMarkers, queue_size=10)

    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        writable.clear()
        msg = copy.deepcopy(shared_msg)
        writable.set()

        if msg is not None:
            for marker in msg.markers:
                pose = marker.pose.pose
                pos = pose.position
                ori = pose.orientation
                cam_to_marker_mat = numpy.dot(translation_matrix((pos.x, pos.y, pos.z)), quaternion_matrix((ori.x, ori.y, ori.z, ori.w)))
                base_to_marker = numpy.dot(base_to_cam_mat, cam_to_marker_mat)
                
                broadcaster.sendTransform(
                    translation_from_matrix(base_to_marker),
                    quaternion_from_matrix(base_to_marker),
                    rospy.Time.now(),
                    'raw_marker_%s'%marker.id,
                    'base', 
                )
                
                if marker.id in handcoded_marker_compensation:
                    base_to_marker = numpy.dot(handcoded_marker_compensation[marker.id], base_to_marker)
                base_to_marker = transform_into_baxter_picking_space(base_to_marker) 
                trans = translation_from_matrix(base_to_marker)
                quat = quaternion_from_matrix(base_to_marker)
                broadcaster.sendTransform(
                    trans,
                    quat,
                    rospy.Time.now(),
                    'baxter_picking_pose_%s'%marker.id,
                    'base', 
                )
                marker.pose.pose.position.x = trans[0]
                marker.pose.pose.position.y = trans[1]
                marker.pose.pose.position.z = trans[2]
                marker.pose.pose.orientation.x = quat[0]
                marker.pose.pose.orientation.y = quat[1]
                marker.pose.pose.orientation.z = quat[2]
                marker.pose.pose.orientation.w = quat[3]
            pub.publish(msg)

        try:
            r.sleep()
        except rospy.ROSInterruptException:
            break
