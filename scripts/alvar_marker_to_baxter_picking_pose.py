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

comfortable_pick_frame = numpy.matrix([[0.9270661704094123, 0.37483076461918924, 0.007086153923013515, 0.746], [0.37479879099742724, -0.9270901506257341, 0.0054515025104288975, -0.089], [0.008612894362151227, -0.002398021632153474, -0.9999600329727978, 0.115], [0.0, 0.0, 0.0, 1.0]])

to_confortable_pick_frame = comfortable_pick_frame[:3, :3].I

handcoded_marker_compensation = {
    0: numpy.array(
        [[0.04258225549353267, -0.9976699860199164, -0.05330431982591899, -0.003], [0.9979499876999266, 0.045024270145620604, -0.045482272893637384, 0.0], [0.04777628665771998, -0.05125830754984532, 0.9975419852519115, -0.033], [0.0, 0.0, 0.0, 1.0]]

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
        [[0.04157746411335095, -0.9972777195581611, -0.06089716373343029, 0.0], [0.9963386576809764, 0.04594110483626879, -0.07210197013183828, -0.011], [0.07470337133203928, -0.0576763812950862, 0.9955364590773818, -0.028], [0.0, 0.0, 0.0, 1.0]]

    , dtype=numpy.float64),
    
    11: numpy.array((
        ((1.0, 0.0, 0.0, 0.016),
        (0.0, 1.0, 0.0, -0.021),
        (0.0, 0.0, 1.0, 0.029),
        (0.0, 0.0, 0.0, 1.0))
    ), dtype=numpy.float64),
    
    13: numpy.array((
[[0.09622387309747282, -0.9940084038491953, -0.05184842643028589, 0.004], [0.9940084038491953, 0.09867616353674313, -0.04701391099286735, 0.0], [0.05184842643028589, -0.04701391099286735, 0.9975477095607297, -0.017], [0.0, 0.0, 0.0, 1.0]]
    ), dtype=numpy.float64),
   
   17: numpy.array((
        ((1.0, 0.0, 0.0, 0.0),
        (0.0, 1.0, 0.0, -0.0),
        (0.0, 0.0, 1.0, 0.075),
        (0.0, 0.0, 0.0, 1.0))
    ), dtype=numpy.float64),    
   
   18: numpy.array((
[[-0.07792074091621037, 0.9962448437817034, -0.037743467957233004, 0.009], [-0.9936061393855619, -0.07450042029363613, 0.08483234731746714, -0.027], [0.081701884374772, 0.044112340840647246, 0.9956801210605593, -0.020], [0.0, 0.0, 0.0, 1.0]]

    ), dtype=numpy.float64),
   
    20: numpy.array((
[[0.06595220199306795, -0.9974408579981445, -0.02760510547328114, 0.019], [0.9958555310795135, 0.06406061873788338, 0.06456003675076039, 0.016], [-0.06262641831212726, -0.031748573556066424, 0.9975319342289496, -0.050], [0.0, 0.0, 0.0, 1.0]]

    ), dtype=numpy.float64),

    22: numpy.array((
        ((1.0,0.0,0.0,0.0),
        (0.0, 1.0, 0.0,  0.016),
        (0.0 , 0.0 ,1.0, 0.016),
        (0.0, 0.0, 0.0, 1.0))
    ), dtype=numpy.float64),
   24: numpy.array((
[[0.06595220199306795, -0.9974408579981445, -0.02760510547328114, 0.019], [0.9958555310795135, 0.06406061873788338, 0.06456003675076039, 0.016], [-0.06262641831212726, -0.031748573556066424, 0.9975319342289496, -0.06], [0.0, 0.0, 0.0, 1.0]]
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
        mat[:3, 1] = -mat[:3, 1]

    return mat

def add_noise(mat):
    #mat[0, 3] += numpy.random.normal(0, 0.02)
    #mat[1, 3] += numpy.random.normal(0, 0.02)
    #mat[2, 3] += numpy.random.normal(0, 0.02)
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
                
                flipped_mat = transform_into_baxter_picking_space(base_to_marker) 
                trans = translation_from_matrix(flipped_mat)
                quat = quaternion_from_matrix(flipped_mat)
                broadcaster.sendTransform(
                    trans,
                    quat,
                    rospy.Time.now(),
                    'flipped_%s'%marker.id,
                    'base', 
                )

                if marker.id in handcoded_marker_compensation:
                    compensated_mat = numpy.dot(flipped_mat, handcoded_marker_compensation[marker.id])
                trans = translation_from_matrix(compensated_mat)
                quat = quaternion_from_matrix(compensated_mat)
                broadcaster.sendTransform(
                    trans,
                    quat,
                    rospy.Time.now(),
                    'compensated_%s'%marker.id,
                    'base', 
                )
            
                noisy_mat = add_noise(compensated_mat)
                trans = translation_from_matrix(noisy_mat)
                quat = quaternion_from_matrix(noisy_mat)
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

            if len(msg.markers) != 0:
                pub.publish(msg)

        try:
            r.sleep()
        except rospy.ROSInterruptException:
            break
