from geometry_msgs.msg import (
    Pose,
    Quaternion,
)

place_pose = Pose()
place_pose.position.x = 0.526379460142
place_pose.position.y = -0.818221808593
place_pose.position.z = 0.0342286718843
place_pose.orientation.x= 0.999937797296
place_pose.orientation.y= -0.0043522273979
place_pose.orientation.z= -0.00938816830392
place_pose.orientation.w= 0.00416196480575

reverting_statistics = {
    'MoveToPrePickPoseWithEmptyHand': {
        "human_collision":{'MoveToPrePickPoseWithEmptyHand':25},
                                      },
    'Pick': {
        "tool_collision": {
            'MoveToPrePickPoseWithEmptyHand': 25,

        },
       "human_collision": {
            'Pick': 25,
        },
    },
    'MoveToPrePickPoseWithFullHand': {
       "human_collision": {
            'MoveToPrePickPoseWithFullHand': 25,
        },
       "object_slip": {
            'Pick': 20,
            "MoveToPrePickPoseWithEmptyHand":5,
        },
       "no_object": {
            'Pick': 24,
            'MoveToPrePickPoseWithEmptyHand': 1,
        },
    },
    'MoveToPrePlacePoseWithFullHand': {
        "object_slip": {
            'Pick': 25,                  
        },
        "human_collision":{
            'MoveToPrePlacePoseWithFullHand':25,
        },
    },
    'Place': {
        "human_collision": {
           'Place': 25,
        },
    },
}
