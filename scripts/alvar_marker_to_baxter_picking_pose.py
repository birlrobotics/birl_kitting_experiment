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
[[-0.9992617534256439, -0.029115724652033777, -0.025064371500081027, -0.007], [0.030844301996866948, -0.996956983632533, -0.07159191169850729, 0.008], [-0.022903649819039557, -0.07231215225885446, 0.9971190377586114, -0.053], [0.0, 0.0, 0.0, 1.0]]

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
[[0.09874775527834512, -0.9935008059000685, -0.05661297999048119, 0.01], [0.9950966080206056, 0.098907735440805, -2.399702436898729e-05, 0.016], [0.0056233027104639, -0.05633301470617644, 0.9983961988713399, -0.009], [0.0, 0.0, 0.0, 1.0]]

    , dtype=numpy.float64),
    
    11: numpy.array((
        ((1.0, 0.0, 0.0, 0.016),
        (0.0, 1.0, 0.0, -0.021),
        (0.0, 0.0, 1.0, 0.029),
        (0.0, 0.0, 0.0, 1.0))
    ), dtype=numpy.float64),
    
    13: numpy.array((
[[0.9966609014365727, 0.07597899709004262, 0.029903838362821362, 0.003], [-0.07591897734354601, 0.9971090488770806, -0.003139032741772042, -0.01], [-0.03005588838727941, 0.0008582823749013425, 0.999547851243059, -0.012], [0.0, 0.0, 0.0, 1.0]]

    ), dtype=numpy.float64),
   
   17: numpy.array((
        ((1.0, 0.0, 0.0, 0.0),
        (0.0, 1.0, 0.0, -0.0),
        (0.0, 0.0, 1.0, 0.075),
        (0.0, 0.0, 0.0, 1.0))
    ), dtype=numpy.float64),    
   
   18: numpy.array((
[[-0.07792074091621037, 0.9962448437817034, -0.037743467957233004, 0.009], [-0.9936061393855619, -0.07450042029363613, 0.08483234731746714, -0.027], [0.081701884374772, 0.044112340840647246, 0.9956801210605593, -0.028], [0.0, 0.0, 0.0, 1.0]]

    ), dtype=numpy.float64),
   
    20: numpy.array((
[[0.06595220199306795, -0.9974408579981445, -0.02760510547328114, 0.019], [0.9958555310795135, 0.06406061873788338, 0.06456003675076039, 0.016], [-0.06262641831212726, -0.031748573556066424, 0.9975319342289496, -0.057], [0.0, 0.0, 0.0, 1.0]]

    ), dtype=numpy.float64),

    22: numpy.array((
        ((1.0,0.0,0.0,0.0),
        (0.0, 1.0, 0.0,  0.016),
        (0.0 , 0.0 ,1.0, 0.016),
        (0.0, 0.0, 0.0, 1.0))
    ), dtype=numpy.float64),
   24: numpy.array((
[[-0.03058437465608832, -0.9980370774263904, -0.05464968535211549, -0.001], [0.9990295438856261, -0.028789531079607356, -0.033333666823407, 0.003], [0.03169489660140265, -0.05561613958560523, 0.9979490360469421, -0.055], [0.0, 0.0, 0.0, 1.0]]
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
