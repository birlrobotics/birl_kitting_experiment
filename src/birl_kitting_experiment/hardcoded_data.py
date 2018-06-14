from geometry_msgs.msg import (
    Pose,
    Quaternion,
)

place_pose = Pose()
place_pose.position.x = 0.441
place_pose.position.y = -0.635
place_pose.position.z = -0.010
place_pose.orientation.x= 0.999
place_pose.orientation.y= -0.046
place_pose.orientation.z= 0.04
place_pose.orientation.w= 0.004

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
            'MoveToPrePickPoseWithEmptyHand': 25,                  
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
