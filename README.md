# birl_kitting_experiment

# How To Run

1. Launch real or sim Baxter

1. Launch camera image publication:

    ```bash
    [Open a new terminal]
    roslaunch openni2_launch openni2.launch
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
    roslaunch birl_moveit_r_ft_config birl_baxter_grippers.launch
    ```

1. Setup sensors, download and install [birl_sensors](https://github.com/birlrobotics/birl_sensors.git). Then run:

    ```bash
    [Open a new terminal]
    rosrun robotiq_force_torque_sensor rq_sensor
    
    [Open a new terminal]
    rosrun tactilesensors4 PollData4
    python  ../tactilesensors4/scripts/tactile_preprocessing.py
    ```

1. Run the experiment:

    ```bash
    [Open a new terminal]
    rosrun birl_kitting_experiment smach_based_kitting_experiment_runner.py
    ```

If you want to run the experiment again, repeat the last step.
