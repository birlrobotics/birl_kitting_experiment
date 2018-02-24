#!/usr/bin/env python  

from visualization_msgs.msg import (
    Marker
)
import sys,os,rospy,tf
from birl_baxter_dmp.dmp_generalize import dmp_imitate
from train_demontration_for_w import get_parameter_w
import baxter_interface
from geometry_msgs.msg import PoseStamped, Pose
import util

DIR_PATH = os.path.dirname(os.path.realpath(__file__))
DATA_PATH = DIR_PATH + "/data/go_hover_2.csv"  # change this name with other names in /data
TRAIN_LEN = 40
TAU = 100.0/TRAIN_LEN # set tau to 1 will give you 100 points
LIMB_NAME = "right"
N_Demo=7   # number of DOFs
N_BFS=500  #number of basis function



def GetEndPointPosition(Arm_right):
    for i in range(50):
        POSE = Arm_right.endpoint_pose()
        EndPoint = [POSE['position'].x ,POSE['position'].y, POSE['position'].z,POSE['orientation'].x,POSE['orientation'].y,POSE['orientation'].z,POSE['orientation'].w]
    return EndPoint

def main():
    rospy.init_node("test_dmp",anonymous=True)
    Arm_right = baxter_interface.Limb('right')
    listener = tf.TransformListener()
    listener.waitForTransform("/base", "/ar_marker_8_pick_frame", rospy.Time(), rospy.Duration(4.0))
    (trans,rot) = listener.lookupTransform("/base", "/ar_marker_8_pick_frame", rospy.Time(0))
    current_pose = GetEndPointPosition(Arm_right)
    target_pose = [trans[0],trans[1],trans[2],rot[0],rot[1],rot[2],rot[3]]
    marker_pub = rospy.Publisher("/visualization_marker", Marker, queue_size=1000)
    dmp_weight,demo_start_pose,demo_end_pose = get_parameter_w(limb_name="right",data_path=DATA_PATH, distance=0.01)
    dmp_traj = dmp_imitate(starting_pose=current_pose,ending_pose=target_pose,weight_mat=dmp_weight,tau=TAU)
    for idx in range(TRAIN_LEN):
        traj_pose = Pose()
        traj_pose.position.x = dmp_traj[idx, 0]
        traj_pose.position.y = dmp_traj[idx, 1]
        traj_pose.position.z = dmp_traj[idx, 2]
        traj_pose.orientation.x = dmp_traj[idx, 3]
        traj_pose.orientation.y = dmp_traj[idx, 4]
        traj_pose.orientation.z = dmp_traj[idx, 5]
        traj_pose.orientation.w =dmp_traj[idx, 6]
        # visualize the pose in rviz using marker
        alpha = float(idx) / TRAIN_LEN * 0.5 + 0.5
        rgba_tuple = ((0.5 * idx), 0.5, (0.5 * idx), alpha)
        rospy.sleep(0.1) # for marker can be showed
        util.send_traj_point_marker(marker_pub=marker_pub, pose=traj_pose, id=idx, rgba_tuple=rgba_tuple)

if __name__ == "__main__":
    main()
