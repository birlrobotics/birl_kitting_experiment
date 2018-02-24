#!/usr/bin/env python

import sys
import rospy
from geometry_msgs.msg import PoseStamped, Pose
import copy
import util


def IK_MoveIt_p2p(MoveIt_arm, StartPosition, EndPosition ,Accuracy):

    waypoints = []  

    wpose = Pose()
    wpose.position.x = StartPosition[0]
    wpose.position.y = StartPosition[1]
    wpose.position.z = StartPosition[2]
    wpose.orientation.x = StartPosition[3]
    wpose.orientation.y = StartPosition[4]
    wpose.orientation.z = StartPosition[5]
    wpose.orientation.w = StartPosition[6]
    waypoints.append(copy.deepcopy(wpose)) # first points
    
    wpose.position.x = EndPosition[0] # second points
    wpose.position.y = EndPosition[1]
    wpose.position.z = EndPosition[2]
    wpose.orientation.x = EndPosition[3]
    wpose.orientation.y = EndPosition[4]
    wpose.orientation.z = EndPosition[5]
    wpose.orientation.w = EndPosition[6]

    waypoints.append(copy.deepcopy(wpose))

    fraction = 0.0
    maxtries = 3
    attempts = 0
    while fraction < 1.0 and attempts < maxtries:
        (plan, fraction) = MoveIt_arm.compute_cartesian_path(
            waypoints,  # waypoints to follow
            Accuracy,  # eef_step
            0.0,   # jump_threshold
            True)  # avoid_collisions
    
    # Increment the number of attempts 
        attempts += 1
     # Print out a progress message
        if attempts % 10 == 0:
            rospy.loginfo("Still trying after " + str(attempts) + " attempts...")
                     
        # If we have a complete plan, execute the trajectory
        if fraction == 1.0:
            rospy.loginfo("Path computed successfully. Moving the arm.")

            MoveIt_arm.execute(plan)
                        
            rospy.loginfo("Path execution complete.")
        else:
            rospy.loginfo("Path planning failed with only " + str(fraction) + " success after " + str(maxtries) + " attempts.")   
                          
    
def IK_MoveIt_ListofPose(MoveIt_arm, TRAIN_LEN, dmp_traj,marker_pub,desired_orien):
    '''keep orientation the same '''
    list_of_poses = []
    fixed_orientation_x = desired_orien[0]
    fixed_orientation_y = desired_orien[1]
    fixed_orientation_z = desired_orien[2]
    fixed_orientation_w = desired_orien[3]
    
    for idx in range(TRAIN_LEN):
        traj_pose = Pose()
        traj_pose.position.x = dmp_traj[idx, 0]
        traj_pose.position.y = dmp_traj[idx, 1]
        traj_pose.position.z = dmp_traj[idx, 2]
        traj_pose.orientation.x = fixed_orientation_x
        traj_pose.orientation.y = fixed_orientation_y
        traj_pose.orientation.z = fixed_orientation_z
        traj_pose.orientation.w = fixed_orientation_w
        # visualize the pose in rviz using marker
#        alpha = float(idx) / TRAIN_LEN * 0.5 + 0.5
#        rgba_tuple = ((0.5 * idx), 0.5, (0.5 * idx), alpha)
#        rospy.sleep(0.1) # for marker can be showed
#        util.send_traj_point_marker(marker_pub=marker_pub, pose=traj_pose, id=idx, rgba_tuple=rgba_tuple)
#            rospy.loginfo("add one pose")
        list_of_poses.append(traj_pose)
  
    fraction = 0.0
    maxtries = 3
    attempts = 0
    while fraction < 1.0 and attempts < maxtries:
        (plan, fraction) = MoveIt_arm.compute_cartesian_path(
            list_of_poses,  # waypoints to follow
            0.01,  # eef_step
            0.0,   # jump_threshold
            True)  # avoid_collisions
    
    # Increment the number of attempts 
        attempts += 1
     # Print out a progress message
        if attempts % 10 == 0:
            rospy.loginfo("Still trying after " + str(attempts) + " attempts...")
                     
        # If we have a complete plan, execute the trajectory
        if fraction == 1.0:
            rospy.loginfo("Path computed successfully. Moving the arm.")

            MoveIt_arm.execute(plan)
                        
            rospy.loginfo("Path execution complete.")
        else:
            rospy.loginfo("Path planning failed with only " + str(fraction) + " success after " + str(maxtries) + " attempts.")  

def IK_calculate(MoveIt_arm,Position,Quaternion):

    # MoveIt_arm : MoveIt_left_arm  or MoveIt_right_arm

    #Start a node
    #rospy.init_node('moveit_node')

    #Create a goal pose for the left/right arm
    arm_goal_pose = PoseStamped()
    arm_goal_pose.header.frame_id = "base"

    #x, y, and z position
    arm_goal_pose.pose.position.x = Position[0]
    arm_goal_pose.pose.position.y = Position[1]
    arm_goal_pose.pose.position.z = Position[2]

    #Orientation as a quaternion
    arm_goal_pose.pose.orientation.x = Quaternion[0]
    arm_goal_pose.pose.orientation.y = Quaternion[1]
    arm_goal_pose.pose.orientation.z = Quaternion[2]
    arm_goal_pose.pose.orientation.w = Quaternion[3]

    #Set the goal state to the pose you just defined
    MoveIt_arm.set_pose_target(arm_goal_pose)

    #Set the start state for the left/right arm
    MoveIt_arm.set_start_state_to_current_state()

    #Plan a path
    IK_plan = MoveIt_arm.plan()

    return IK_plan 


def IK_Execute(MoveIt_arm,IK_plan):
    #Execute the plan
    MoveIt_arm.execute(IK_plan)



def FK_calculate(MoveIt_arm,JointAngle):

    
    
    MoveIt_arm.set_joint_value_target(JointAngle)

    #Set the start state for the left/right arm
    MoveIt_arm.set_start_state_to_current_state()

    #Plan a path
    IK_plan = MoveIt_arm.plan()

    MoveIt_arm.execute(IK_plan)


    # .65376956364916, 0.15558032315443168