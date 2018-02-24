#!/usr/bin/env python

import rospy

import baxter_interface
from baxter_interface import CHECK_VERSION

import moveit_commander
import IK

import time
import sys
import tf
from std_msgs.msg import String, Int16MultiArray
from geometry_msgs.msg import Pose,Vector3,Quaternion
from sensor_msgs.msg import Image
import ipdb
import math,os
from birl_baxter_dmp.dmp_generalize import dmp_imitate
from train_demontration_for_w import get_parameter_w
from numpy import linalg as LA
from visualization_msgs.msg import (
    Marker
)
import util


DIR_PATH = os.path.dirname(os.path.realpath(__file__))
DATA_PATH = DIR_PATH + "/data/go_hover_2.csv"  # change this name with other names in /data
TRAIN_LEN = 40
TAU = 100.0/TRAIN_LEN

def callback1(msg):
    # Callback for found_markers
    pass

class BuildHouse(object):     
    
    def __init__(self):

        ##### Initialize AR_tag transforms #####
        rospy.init_node('master', anonymous=True)
        self.listener = tf.TransformListener()

        ##### Initialize Inverse Kinematics #####  
        self.Counting = 0
        #Initialize moveit_commander
        moveit_commander.roscpp_initialize('/joint_states:=/robot/joint_states')
        #Initialize MoveIt for both arms
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.MoveIt_right_arm = moveit_commander.MoveGroupCommander('right_arm')
        self.MoveIt_right_arm.set_goal_position_tolerance(0.01)
        self.MoveIt_right_arm.set_goal_orientation_tolerance(0.01) 
        self.MoveIt_right_gripper = moveit_commander.MoveGroupCommander('right_hand')
        ##### Initialize Gripper control #####
        self.Arm_right = baxter_interface.Limb('right')
        self.Gripper_right = baxter_interface.Gripper('right', CHECK_VERSION)
        self.end_effector_link = self.MoveIt_right_arm.get_end_effector_link()

        ##### Define the desired position/orientation for the first box  #####
        self.Desired_Position = [ 0.116669104195, -0.798563848794, 0.204631328968]    
        self.Desired_Quaternion = [0.998805408768,0.0488203611017,0.00166462220289,0.00124771431711]
        self.moved_blocks = {}

        ##### Verify robot is enabled  #####
        print("Getting robot state... ")
        self._rs = baxter_interface.RobotEnable(CHECK_VERSION)
        self._init_state = self._rs.state().enabled
        print("Enabling robot... ")
        self._rs.enable()
        print("Running. Ctrl-c to quit")

        # publish a string of traj for visuallizing
        self.marker_pub = rospy.Publisher("/visualization_marker", Marker, queue_size=1000)
    

    def GetEndPointPosition(self):
        for i in range(50):
            POSE = self.Arm_right.endpoint_pose()
            EndPoint = [POSE['position'].x ,POSE['position'].y, POSE['position'].z,POSE['orientation'].x,POSE['orientation'].y,POSE['orientation'].z,POSE['orientation'].w]
        return EndPoint


    def GetTargetBlock(self,detectedMarkers):
        # sort box id, first pick box with small id number
        orderedMarkers = []
        for num in detectedMarkers:
            orderedMarkers.append(num)
            if num not in self.moved_blocks:
                self.moved_blocks[num] = 0
        orderedMarkers.sort()
        print orderedMarkers

        chosenMarker = -1
        for i in xrange(len(orderedMarkers)):
            if self.moved_blocks[orderedMarkers[i]] == 0:
                chosenMarker = orderedMarkers[i]
                break

        if chosenMarker != -1:
            self.moved_blocks[chosenMarker] = 1
        return chosenMarker

    def FindBlock(self):
#        ipdb.set_trace()
        print "==============="
        print "FIND BLOCK"
        print "==============="
        #Get information about the markers found
        rospy.Subscriber("/demo/found_markers",Int16MultiArray, callback1)
        msg1 = rospy.wait_for_message('/demo/found_markers',Pose)
        print msg1.data
        detectedMarkers = msg1.data

        chosenMarker = self.GetTargetBlock(detectedMarkers)
        if chosenMarker != -1: # check if there is box 
            if chosenMarker == 0: # marker_id = 0 
                try:
                    targetMarker = 'ar_marker_'+str(chosenMarker)+"_pick_frame"
                    print "the marker frame chosen", targetMarker        
                    self.listener.waitForTransform("/base", targetMarker, rospy.Time(), rospy.Duration(4.0))
                    (trans,rot) = self.listener.lookupTransform('/base',targetMarker,rospy.Time(0))
                    off_set = 0.01 # set offset 
                    Px = trans[0]
                    Py = trans[1] + off_set
                    Pz = trans[2] 
                    Qx = rot[0]
                    Qy = rot[1]
                    Qz = rot[2]
                    Qw = rot[3]
                    self.pose = [Px,Py,Pz,Qx,Qy,Qz,Qw]
                    print self.pose
                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                    pass
            
            if chosenMarker == 4:
                try:
                    targetMarker = 'ar_marker_'+str(chosenMarker)+"_pick_frame"
                    print "the marker frame chosen", targetMarker        
                    self.listener.waitForTransform("/base", targetMarker, rospy.Time(), rospy.Duration(4.0))
                    (trans,rot) = self.listener.lookupTransform('/base',targetMarker,rospy.Time(0))
                    off_set = -0.05
                    Px = trans[0]
                    Py = trans[1] + off_set
                    Pz = trans[2] 
                    Qx = rot[0]
                    Qy = rot[1]
                    Qz = rot[2]
                    Qw = rot[3]
                    self.pose = [Px,Py,Pz,Qx,Qy,Qz,Qw]
                    print self.pose
                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                    pass
            
            if chosenMarker == 8:
                try:
                    targetMarker = 'ar_marker_'+str(chosenMarker)+"_pick_frame"
                    print "the marker frame chosen", targetMarker        
                    self.listener.waitForTransform("/base", targetMarker, rospy.Time(), rospy.Duration(4.0))
                    (trans,rot) = self.listener.lookupTransform('/base',targetMarker,rospy.Time(0))
                    off_set = -0.01
                    Px = trans[0]
                    Py = trans[1] + off_set
                    Pz = trans[2] 
                    Qx = rot[0]
                    Qy = rot[1]
                    Qz = rot[2]
                    Qw = rot[3]
                    self.pose = [Px,Py,Pz,Qx,Qy,Qz,Qw]
                    print self.pose
                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                    pass
            if chosenMarker == 11:
                try:
                    targetMarker = 'ar_marker_'+str(chosenMarker)+"_pick_frame"
                    print "the marker frame chosen", targetMarker        
                    self.listener.waitForTransform("/base", targetMarker, rospy.Time(), rospy.Duration(4.0))
                    (trans,rot) = self.listener.lookupTransform('/base',targetMarker,rospy.Time(0))
                    off_set = -0.01
                    Px = trans[0]
                    Py = trans[1] 
                    Pz = trans[2]
                    Qx = rot[0]
                    Qy = rot[1]
                    Qz = rot[2]
                    Qw = rot[3]
                    self.pose = [Px,Py,Pz,Qx,Qy,Qz,Qw]
                    print self.pose
                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                    pass
            
            if chosenMarker == 13:
                try:
                    targetMarker = 'ar_marker_'+str(chosenMarker)+"_pick_frame"
                    print "the marker frame chosen", targetMarker        
                    self.listener.waitForTransform("/base", targetMarker, rospy.Time(), rospy.Duration(4.0))
                    (trans,rot) = self.listener.lookupTransform('/base',targetMarker,rospy.Time(0))
                    off_set = 0
                    Px = trans[0]
                    Py = trans[1] + off_set
                    Pz = trans[2] 
                    Qx = rot[0]
                    Qy = rot[1]
                    Qz = rot[2]
                    Qw = rot[3]
                    self.pose = [Px,Py,Pz,Qx,Qy,Qz,Qw]
                    print "pick marker13"
                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                    pass
        else:
                    user_input = raw_input('No blocks left. Please give me more blocks. Press ''Enter'' when ready!!!')
                    if user_input == 's':
                        moveit_commander.roscpp_shutdown()
                    else:   
                        self.FindBlock()

    def MoveBlock(self):

        print "MoveBlock Step(1):  Approach to Taget Block (Approximately)"
        
        StartPosition = self.GetEndPointPosition()
        EndPosition = [self.pose[0],self.pose[1] ,self.pose[2] + 0.2,self.pose[3],self.pose[4],self.pose[5],self.pose[6]]
        Accuracy = 0.01        # Rough
        
        IK.IK_MoveIt_p2p(self.MoveIt_right_arm, StartPosition,EndPosition , Accuracy)  # go to hover position
#        ipdb.set_trace()
        rospy.sleep(1)
        self.Gripper_control(self.Gripper_right,'open')
        rospy.sleep(1)
        self.MoveIt_right_arm.shift_pose_target(2, -0.2, self.end_effector_link) # go down 2cm
        self.MoveIt_right_arm.go()
        rospy.sleep(1)
        self.Gripper_control(self.Gripper_right,'close')
        rospy.sleep(1)
        self.MoveIt_right_arm.shift_pose_target(2, 0.1, self.end_effector_link) # go up 2cm
        self.MoveIt_right_arm.go()
        
        Disired_Pose = [self.Desired_Position[0], self.Desired_Position[1],self.Desired_Position[2],
                        self.Desired_Quaternion[0], self.Desired_Quaternion[1], self.Desired_Quaternion[2], self.Desired_Quaternion[3]]
        
        dmp_traj = self.Motion_Generation_Dmp(Disired_Pose)
        IK.IK_MoveIt_ListofPose(self.MoveIt_right_arm, TRAIN_LEN, dmp_traj,self.marker_pub,self.Desired_Quaternion)
        rospy.sleep(1)
        self.MoveIt_right_arm.shift_pose_target(2, -0.2, self.end_effector_link)
        self.MoveIt_right_arm.go()
        rospy.sleep(1)
        self.Gripper_control(self.Gripper_right,'open')
        rospy.sleep(1)
        self.MoveIt_right_arm.shift_pose_target(2, 0.2, self.end_effector_link)
        self.MoveIt_right_arm.go()
        rospy.sleep(1)
        self.MoveRightArmHome() 
        
        
    def Motion_Generation_Dmp(self,Disired_Pose):
        marker_pub = rospy.Publisher("/visualization_marker", Marker, queue_size=1000)
        current_pose = self.GetEndPointPosition()
        target_pose = Disired_Pose
        dmp_weight,demo_start_pose,demo_end_pose = get_parameter_w(limb_name="right",data_path=DATA_PATH, distance=0.01)
        dmp_traj = dmp_imitate(starting_pose=current_pose,ending_pose=target_pose,weight_mat=dmp_weight,tau=TAU)
#        for idx in range(TRAIN_LEN):
#            traj_pose = Pose()
#            traj_pose.position.x = self.dmp_traj[idx, 0]
#            traj_pose.position.y = self.dmp_traj[idx, 1]
#            traj_pose.position.z = self.dmp_traj[idx, 2]
#            traj_pose.orientation.x = self.dmp_traj[idx, 3]
#            traj_pose.orientation.y = self.dmp_traj[idx, 4]
#            traj_pose.orientation.z = self.dmp_traj[idx, 5]
#            traj_pose.orientation.w =self.dmp_traj[idx, 6]
#            # visualize the pose in rviz using marker
#            alpha = float(idx) / TRAIN_LEN * 0.5 + 0.5
#            rgba_tuple = ((0.5 * idx), 0.5, (0.5 * idx), alpha)
#            rospy.sleep(0.1) # for marker can be showed
#            util.send_traj_point_marker(marker_pub=marker_pub, pose=traj_pose, id=idx, rgba_tuple=rgba_tuple)

        return dmp_traj

    def UpdateDesiredPosition(self):
        #z component dependent on block size
        self.Desired_Position = [self.Desired_Position[0]+0.15, self.Desired_Position[1],self.Desired_Position[2]]  


    def Gripper_control(self,Gripper,command):
    
        if command == 'open':
           Gripper.open()                  
        elif command == 'close':
            Gripper.close()
        else:
            print('The gripper command is not valid')

    def MoveRightArmHome(self):
        joint_positions = [  0.0395000052880494,-1.1623739420201722, 0.9111845880039358, 1.1965050145506226,
                           -0.38502917775923884, 1.5968740001887156, 0.004985437560627594]
        self.MoveIt_right_arm.set_joint_value_target(joint_positions)
        self.MoveIt_right_arm.go()     

    def MainFunction(self):
        
        self.MoveRightArmHome()     
        raw_input('Everything is Ready. Press ''Enter'' to go!!!')

        ### While Loop ###
        while not rospy.is_shutdown():
            if not self._rs.state().enabled:
                rospy.logerr("Control rate timeout.")
                break
            self.Counting =  self.Counting + 1
            self.FindBlock()
            self.MoveBlock()          # Left arm apJointAngleproach to the target; gripper close; move to desired position; gripper open
            self.UpdateDesiredPosition()   # Update the desired position (expecially the height) for the next target


def main():
    # initialization
    BH = BuildHouse()
    # run main function
    BH.MainFunction()
    

if __name__ == "__main__":
    main()




