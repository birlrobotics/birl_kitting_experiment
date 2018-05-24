# birl_kitting_experiment

# How To Run

1. Launch real or sim Baxter

1. Launch camera image publication:

    ```bash
    [Open a new terminal]
    roslaunch openni2_launch openni2.launch
    ```
    OR (use the left hand camera as input) 
    ```
    rosrun birl_kitting_experiment setup_baxter_left_hand_camera.py
    rosrun birl_kitting_experiment set_left_arm2static_pose.py
    ```

1. Launch alvar recognition with bundles, set up camera-robot transform:

    ```bash
    [Open a new terminal]
    roslaunch birl_kitting_experiment alvar_marker_demo.launch
    ```

1. Publish available Baxter picking poses via topic(/baxter_available_picking_pose) and TF(/baxter_picking_pose_[marker id]):

    ```bash
    [Open a new terminal]
    rosrun birl_kitting_experiment alvar_marker_to_baxter_picking_pose.py
    ```

1. Setup motion services:

    ```bash
    [Open a new terminal]
    rosrun birl_baxter_online_urdf_update update_urdf.py
    rosrun baxter_interface joint_trajectory_action_server.py
    roslaunch birl_moveit_r_ft_config birl_baxter_gripper.launch
    ```

1. Setup sensors, download and install [birl_sensors](https://github.com/birlrobotics/birl_sensors.git). Then run:

    ```bash
    [Open a new terminal]
    rosrun robotiq_force_torque_sensor rq_sensor
    
    [Open a new terminal]
    rosrun tactilesensors4 PollData4
    rosrun tactilesensors4 tactile_static.py
    ```

1.ssh command:
    ```
    [open a new terminal]
     ssh ruser@011405P0002.local
    ```

1. Run the experiment:

    ```bash
    [Open a new terminal]
    rosrun birl_kitting_experiment smach_based_kitting_experiment_runner.py
    ```

If you want to run the experiment again, repeat the last step.

# How to record/train DMP
1. To record a demonstraion: cd ../birl_kitting_expereiment/scripts/
   ```
   python record_demonstration.py --name "DEMONSTRATION_NAME"
   ```
2. Kinesthetic teaching the robot arm for recording the demonstration until finish, and then press CTRL+C

3.To train a DMP models of all recorded demonstraions: cd ../birl_kitting_expereiment/scripts/
```
python cook_dmp_models_for_smach_states.py
```
4. For manually active the anomal
```
rosrun smach_based_introspection_framework send_manual_anomaly_signal.py
```
