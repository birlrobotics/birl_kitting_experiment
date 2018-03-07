import smach
from geometry_msgs.msg import (
    Pose,
    Quaternion,
)
import copy
import random
from tf.transformations import (
    translation_matrix,
    quaternion_matrix,
)
import baxter_interface
import numpy
import os
import hardcoded_data
import dill
from ar_track_alvar_msgs.msg import AlvarMarkers
import rospy

dir_of_this_script = os.path.dirname(os.path.realpath(__file__))
dmp_model_dir = os.path.join(dir_of_this_script, '..', '..', 'data', 'dmp_models')

pick_hover_height = 0.10
place_step_size = 0.07
place_hover_height = 0.10

class MoveToHomePose(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Successful'])
        self.state_no = 0 # Skill tag
        self.depend_on_prev_state = False # Set this flag accordingly

    def get_joint_state_goal(self):
        name = ['head_nod', 'head_pan', 'left_e0', 'left_e1', 'left_s0', 'left_s1', 'left_w0', 'left_w1', 'left_w2', 'right_e0', 'right_e1', 'right_s0', 'right_s1', 'right_w0', 'right_w1', 'right_w2', 'torso_t0']
        position = [0.0, 0.09433981845495294, 0.4851214241687621, 1.513655542445932, 0.5622039587600042, -0.07094661143970038, -0.5560680356084625, 0.400368985638093, -0.22511168062218448, 0.7412962157456262, 0.9533690596707847, 0.1499466220157992, -0.8563447748370322, -0.3992185000471789, 1.6532477941435046, 0.05253884198507542, -12.565987119160338]
        d = dict(zip(name, position))
        return d

    def after_motion(self):
        from std_srvs.srv import Trigger
        try:
            rospy.wait_for_service('/robotiq_wrench_calibration_service', timeout=3)
            trigger = rospy.ServiceProxy('/robotiq_wrench_calibration_service', Trigger)
            resp = trigger()
            rospy.wait_for_service('/tactile_texel_sum_reset', timeout=3)
            trigger = rospy.ServiceProxy('/tactile_texel_sum_reset', Trigger)
            resp = trigger()
            rospy.sleep(5)
        except Exception as exc:
            rospy.logerr("calling force sensor calibration failed: %s"%exc)

    def determine_successor(self): # Determine next state
        return 'Successful'


class DeterminePickPose(smach.State):
    pick_pose = None
    already_pick_count = 0
    def __init__(self):
        smach.State.__init__(self, outcomes=['GotOneFromVision', 'VisionSaysNone'])
        self.state_no = 0 # Skill tag
        self.depend_on_prev_state = False # Set this flag accordingly

    def determine_successor(self): # Determine next state
        

        msg = rospy.wait_for_message("baxter_available_picking_pose", AlvarMarkers)
        if len(msg.markers) == 0:
            return 'VisionSaysNone'
    
        DeterminePickPose.pick_pose = msg.markers[0].pose.pose

        '''
        if DeterminePickPose.already_pick_count >= 3:
            return 'VisionSaysNone'
        DeterminePickPose.pick_pose = Pose()
        DeterminePickPose.pick_pose.position.x = 0.71911746461+random.uniform(-0.1, +0.1)
        DeterminePickPose.pick_pose.position.y = -0.134129746892+random.uniform(-0.1, +0.1)
        DeterminePickPose.pick_pose.position.z = 0.315673091393+random.uniform(-0.1, +0.1)
        DeterminePickPose.pick_pose.orientation = Quaternion(
            x= -0.25322831688,
            y= 0.966477494136,
            z= 0.0131814413255,
            w= -0.0402855118249,
        )
        '''
        DeterminePickPose.already_pick_count += 1
        return 'GotOneFromVision'

class MoveToPrePickPoseWithEmptyHand(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Successful'])
        self.state_no = 3 # Skill tag
        self.depend_on_prev_state = True # Set this flag accordingly

    def get_dmp_model(self):
        return dill.load(open(os.path.join(dmp_model_dir, 'home_to_pre_pick'), 'r'))
    
    def get_pose_goal(self):
        pose = copy.deepcopy(DeterminePickPose.pick_pose)
        pos = pose.position
        ori = pose.orientation
        base_to_pose_mat = numpy.dot(translation_matrix((pos.x, pos.y, pos.z)), quaternion_matrix((ori.x, ori.y, ori.z, ori.w)))
        pose.position.x -= pick_hover_height*base_to_pose_mat[0, 2]
        pose.position.y -= pick_hover_height*base_to_pose_mat[1, 2]
        pose.position.z -= pick_hover_height*base_to_pose_mat[2, 2]
        return pose

    def determine_successor(self): # Determine next state
        return 'Successful'

class Pick(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Successful'])
        self.state_no = 4 # Skill tag
        self.depend_on_prev_state = True # Set this flag accordingly

    def before_motion(self):
        g = baxter_interface.Gripper('right')
        rospy.sleep(1)
        g.open()

    def after_motion(self):
        g = baxter_interface.Gripper('right')
        rospy.sleep(1)
        g.close()

    def get_dmp_model(self):
        return dill.load(open(os.path.join(dmp_model_dir, 'pre_pick_to_pick'), 'r'))
    
    def get_pose_goal(self):
        pose = copy.deepcopy(DeterminePickPose.pick_pose)
        return pose

    def determine_successor(self): # Determine next state
        return 'Successful'

class MoveToPrePickPoseWithFullHand(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Successful'])
        self.state_no = 5 # Skill tag
        self.depend_on_prev_state = True # Set this flag accordingly

    def get_dmp_model(self):
        return dill.load(open(os.path.join(dmp_model_dir, 'pick_to_pre_pick'), 'r'))
    
    def get_pose_goal(self):
        pose = copy.deepcopy(DeterminePickPose.pick_pose)
        pos = pose.position
        ori = pose.orientation
        base_to_pose_mat = numpy.dot(translation_matrix((pos.x, pos.y, pos.z)), quaternion_matrix((ori.x, ori.y, ori.z, ori.w)))
        pose.position.x -= pick_hover_height*base_to_pose_mat[0, 2]
        pose.position.y -= pick_hover_height*base_to_pose_mat[1, 2]
        pose.position.z -= pick_hover_height*base_to_pose_mat[2, 2]
        return pose

    def determine_successor(self): # Determine next state
        return 'Successful'

class DeterminePlacePose(smach.State):
    place_pose = None
    already_plac_count = 0
    def __init__(self):
        smach.State.__init__(self, outcomes=['Successful'])
        self.state_no = 0 # Skill tag
        self.depend_on_prev_state = False # Set this flag accordingly

    def determine_successor(self): # Determine next state
        DeterminePlacePose.place_pose = hardcoded_data.place_pose

        pos = DeterminePlacePose.place_pose.position
        ori = DeterminePlacePose.place_pose.orientation
        base_to_place_mat = numpy.dot(translation_matrix((pos.x, pos.y, pos.z)), quaternion_matrix((ori.x, ori.y, ori.z, ori.w)))

        step = DeterminePlacePose.already_plac_count*place_step_size

        DeterminePlacePose.place_pose.position.x -= step*base_to_place_mat[0, 0]
        DeterminePlacePose.place_pose.position.y -= step*base_to_place_mat[1, 0]
        DeterminePlacePose.place_pose.position.z -= step*base_to_place_mat[2, 0]

        return 'Successful'

class MoveToPrePlacePoseWithFullHand(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Successful'])
        self.state_no = 7 # Skill tag
        self.depend_on_prev_state = True # Set this flag accordingly

    def get_dmp_model(self):
        return dill.load(open(os.path.join(dmp_model_dir, 'pre_pick_to_pre_place'), 'r'))
    
    def get_pose_goal(self):
        pose = copy.deepcopy(DeterminePlacePose.place_pose)
        pos = pose.position
        ori = pose.orientation
        base_to_pose_mat = numpy.dot(translation_matrix((pos.x, pos.y, pos.z)), quaternion_matrix((ori.x, ori.y, ori.z, ori.w)))
        pose.position.x -= place_hover_height*base_to_pose_mat[0, 2]
        pose.position.y -= place_hover_height*base_to_pose_mat[1, 2]
        pose.position.z -= place_hover_height*base_to_pose_mat[2, 2]
        return pose

    def determine_successor(self): # Determine next state
        return 'Successful'

class Place(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Successful'])
        self.state_no = 8 # Skill tag
        self.depend_on_prev_state = True # Set this flag accordingly

    def after_motion(self):
        baxter_interface.Gripper('right').open()

    def get_dmp_model(self):
        return dill.load(open(os.path.join(dmp_model_dir, 'pre_place_to_place'), 'r'))
    
    def get_pose_goal(self):
        pose = copy.deepcopy(DeterminePlacePose.place_pose)
        return pose

    def determine_successor(self): # Determine next state
        DeterminePlacePose.already_plac_count += 1
        return 'Successful'

class MoveToPrePlacePoseWithEmptyHand(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Successful'])
        self.state_no = 9 # Skill tag
        self.depend_on_prev_state = True # Set this flag accordingly

    def get_dmp_model(self):
        return dill.load(open(os.path.join(dmp_model_dir, 'place_to_pre_place'), 'r'))
    
    def get_pose_goal(self):
        pose = copy.deepcopy(DeterminePlacePose.place_pose)
        pos = pose.position
        ori = pose.orientation
        base_to_pose_mat = numpy.dot(translation_matrix((pos.x, pos.y, pos.z)), quaternion_matrix((ori.x, ori.y, ori.z, ori.w)))
        pose.position.x -= place_hover_height*base_to_pose_mat[0, 2]
        pose.position.y -= place_hover_height*base_to_pose_mat[1, 2]
        pose.position.z -= place_hover_height*base_to_pose_mat[2, 2]
        return pose

    def determine_successor(self): # Determine next state
        return 'Successful'

def assembly_user_defined_sm():
    sm = smach.StateMachine(outcomes=['TaskFailed', 'TaskSuccessful'])
    with sm:
        smach.StateMachine.add(
            MoveToHomePose.__name__,
            MoveToHomePose(),
            transitions={
                'Successful': DeterminePickPose.__name__
            }
        )
        smach.StateMachine.add(
            DeterminePickPose.__name__,
            DeterminePickPose(),
            transitions={
                'GotOneFromVision': MoveToPrePickPoseWithEmptyHand.__name__,
                'VisionSaysNone': 'TaskSuccessful',
            }
        )
        smach.StateMachine.add(
            MoveToPrePickPoseWithEmptyHand.__name__,
            MoveToPrePickPoseWithEmptyHand(),
            transitions={
                'Successful': Pick.__name__
            }
        )
        smach.StateMachine.add(
            Pick.__name__,
            Pick(),
            transitions={
                'Successful': MoveToPrePickPoseWithFullHand.__name__
            }
        )
        smach.StateMachine.add(
            MoveToPrePickPoseWithFullHand.__name__,
            MoveToPrePickPoseWithFullHand(),
            transitions={
                'Successful': DeterminePlacePose.__name__
            }
        )
        smach.StateMachine.add(
            DeterminePlacePose.__name__,
            DeterminePlacePose(),
            transitions={
                'Successful': MoveToPrePlacePoseWithFullHand.__name__
            }
        )
        smach.StateMachine.add(
            MoveToPrePlacePoseWithFullHand.__name__,
            MoveToPrePlacePoseWithFullHand(),
            transitions={
                'Successful': Place.__name__
            }
        )
        smach.StateMachine.add(
            Place.__name__,
            Place(),
            transitions={
                'Successful': MoveToPrePlacePoseWithEmptyHand.__name__
            }
        )
        smach.StateMachine.add(
            MoveToPrePlacePoseWithEmptyHand.__name__,
            MoveToPrePlacePoseWithEmptyHand(),
            transitions={
                'Successful': MoveToHomePose.__name__
            }
        )
    return sm
