# birl_kitting_experiment

# How To Run

1. Launch real or sim Baxter

2. Launch camera image publication:

```bash
roslaunch openni2_launch openni2.launch
```

3. Launch alvar recognition with bundles, set up camera-robot transform:

```bash
roslaunch birl_kitting_experiment alvar_marker_demo.launch
```

4. Publish available Baxter picking poses via topic(/baxter_available_picking_pose) and TF(/baxter_picking_pose_[marker id]):

```bash
rosrun birl_kitting_experiment alvar_marker_to_baxter_picking_pose.py
```

5. Move Baxter right arm to picking poses:
```bash
[Open a new terminal]
rosrun baxter_interface joint_trajectory_action_server.py

[Open a new terminal]
roslaunch baxter_moveit_config baxter_grippers.launch

[Open a new terminal]
rosrun birl_kitting_experiment test_baxter_picking_topic.py 
[Read the console and interact with it]
```
