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

comfortable_pick_frame = numpy.matrix([[0.9291218997620283, 0.36968764383826475, 0.007971284458086786, 0.777544280524], [0.36854106257578606, -0.9275684272500412, 0.06159787305090193, -0.0195243675556], [0.030165884341586742, -0.05429418718607709, -0.9980692163671278, 0.285303310464], [0.0, 0.0, 0.0, 1.0]])
to_confortable_pick_frame = comfortable_pick_frame[:3, :3].I

handcoded_marker_compensation = {
    0: numpy.array(
        ((1.0, 0.0, 0.0, -0.0),
        (0.0, 1.0, 0.0, -0.0),
        (0.0, 0.0, 1.0, 0.051),
        (0.0, 0.0, 0.0, 1.0))
    , dtype=numpy.float64),
    4: numpy.array(
        ((1.0, 0.0, 0.0, -0.0),
        (0.0, 1.0, 0.0, -0.0),
        (0.0, 0.0, 1.0, 0.05),
        (0.0, 0.0, 0.0, 1.0))
    , dtype=numpy.float64),
    
    6: numpy.array(
        ((1.0, 0.0, 0.0, -0.0),
        (0.0, 1.0, 0.0, -0.0),
        (0.0, 0.0, 1.0, 0.05),
        (0.0, 0.0, 0.0, 1.0))
    , dtype=numpy.float64),
    
    8: numpy.array(
        ((1.0, 0.0, 0.0, 0.0),
        (0.0, 1.0, 0.0, 0.0),
        (0.0, 0.0, 1.0, 0.064),
        (0.0, 0.0, 0.0, 1.0))
    , dtype=numpy.float64),
    
    11: numpy.array((
        ((1.0, 0.0, 0.0, 0.016),
        (0.0, 1.0, 0.0, -0.021),
        (0.0, 0.0, 1.0, 0.029),
        (0.0, 0.0, 0.0, 1.0))
    ), dtype=numpy.float64),
    
    13: numpy.array((
        ((1.0, 0.0, 0.0, 0.0),
        (0.0, 1.0, 0.0, -0.0),
        (0.0, 0.0, 1.0, 0.075),
        (0.0, 0.0, 0.0, 1.0))
    ), dtype=numpy.float64),
   
   17: numpy.array((
        ((1.0, 0.0, 0.0, 0.0),
        (0.0, 1.0, 0.0, -0.0),
        (0.0, 0.0, 1.0, 0.075),
        (0.0, 0.0, 0.0, 1.0))
    ), dtype=numpy.float64),    
   
   18: numpy.array((
        ((0.0, -1.0,  0.0,  0.003),
        (1.0, 0.0, 0.0,  -0.007),
        (0.0 , 0.0 ,1.0, 0.39),
        (0.0, 0.0, 0.0, 1.0))
    ), dtype=numpy.float64),
   
    20: numpy.array((
        ((1.0,0.0,0.0,0.0),
        (0.0, 1.0, 0.0,  0.0),
        (0.0 , 0.0 ,1.0, 0.046),
        (0.0, 0.0, 0.0, 1.0))
    ), dtype=numpy.float64),

    22: numpy.array((
        ((1.0,0.0,0.0,0.0),
        (0.0, 1.0, 0.0,  0.016),
        (0.0 , 0.0 ,1.0, 0.016),
        (0.0, 0.0, 0.0, 1.0))
    ), dtype=numpy.float64),
   24: numpy.array((
        ((1.0,0.0,0.0,0.0),
        (0.0, 1.0, 0.0,  0.0),
        (0.0 , 0.0 ,1.0, 0.046),
        (0.0, 0.0, 0.0, 1.0))
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

    # Determine x, y, z by directions
    for axis in range(3):
        swap_with =  abs(mat[:3, axis]).argmax()
        if swap_with != axis:
            tmp = mat[:3, swap_with].copy()
            mat[:3, swap_with] = mat[:3, axis]
            mat[:3, axis] = tmp

    # If z is pointing upwards, flip it
    if mat[:3, 2][2] > 0:
        mat[:3, 2] = -mat[:3, 2]

    # If x is pointing inwards, flip it
    vec_x = mat[:3, 0].reshape((3, -1))
    if (to_confortable_pick_frame*vec_x)[0] < 0:
        mat[:3, 0] = -mat[:3, 0]
        mat[:3, 1] = -mat[:3, 1]

    # Make sure x, y, z subject to right-hand rule
    if numpy.cross(mat[:3, 0], mat[:3, 1])[2] > 0:
        tmp = mat[:3, 0].copy()
        mat[:3, 0] = mat[:3, 1]
        mat[:3, 1] = tmp 

    return mat
        

if __name__ == '__main__':
    rospy.init_node("alvar_marker_to_baxter_picking_pose_py")
    
    rospy.Subscriber("ar_pose_marker", AlvarMarkers, cb)

    writable.set()

    listener = tf.TransformListener()


    broadcaster = tf.TransformBroadcaster()
    pub = rospy.Publisher("baxter_available_picking_pose", AlvarMarkers, queue_size=10)

    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        writable.clear()
        msg = copy.deepcopy(shared_msg)
        writable.set()

        if msg is not None:


            look_up_t = rospy.Time(0)
            listener.waitForTransform('base', 'left_hand_camera', look_up_t, rospy.Duration(3))
            base_to_cam = listener.lookupTransform('base', 'left_hand_camera', look_up_t)
            base_to_cam_mat = listener.fromTranslationRotation(*base_to_cam) 


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
                    base_to_marker = numpy.dot(base_to_marker , handcoded_marker_compensation[marker.id])
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
