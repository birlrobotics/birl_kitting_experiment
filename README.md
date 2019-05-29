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

    . Launch alvar recognition with bundles, set up camera-robot transform:

    ```bash
    [Open a new terminal]
    roslaunch birl_kitting_experiment alvar_marker_demo.launch
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
    
    [Open a new terminal]
    rosrun smach_based_introspection_framework timeseries_publisher.py
    ```
    
1. Publish the state switch and tag_multimodal_topic_and_service
   ```
   rosrun smach_based_introspection_framework tag_multimodal_topic_and_service.py
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
    If you want to run the experiment again, repeat this step.
    ```

1. Open the anomaly detection service
   ```
   rosrun smach_based_introspection_framework anomaly_detection.py
   ```
   
1. Open anomaly classification service
   ```
   rosrun smach_based_introspection_framework redis_based_anomaly_classification.py
   ```

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
# How to record the adaptation (Human-in-the-loop) recovery policy
### The recovery skill is label begin with 1000 and corrspounds the specific skill
1. Whether the valuable in smach_based_instospection_framework/configurables.py is **True** or not.
   ```
   HUMAN_AS_MODEL_MODL = True/False
   ```
2. Pausing the robot movement while robot encounter an anomaly and label it carefully, for instance
   ```
   tool collision
   ```
3. And then following the instruction to record the human teaching behavior:
   ```
   start
   end
   ```
4. extract the adaptation demonstration data:
   ```
   roscd smach_based_introspection_framework
   cd src/smach_based_introspection_framework/offline_part
   python process_experiment_record_to_dataset.py
   python process_dataset_to_models.py (we commonly comment the command for generating introspection and classification models)
   ```
# Citations
If you use this toolbox, please cite the following related papers:

[1] "A Latent State-based Multimodal Execution Monitor with Anomaly Detection and Classification for Robot Introspection", Hongmin Wu, Yisheng Guan , Juan Rojas, Appl. Sci. 2019, 9(6), 1072; doi: 10.3390/app9061072.[http://www.juanrojas.net/files/papers/2019AS-Wu-LatState_MMExecMntr_AnmlyDetClassif.pdf](http://www.juanrojas.net/files/papers/2019AS-Wu-LatState_MMExecMntr_AnmlyDetClassif.pdf);

[2]"Robot Introspection for Process Monitoring in Robotic Contact Tasks through Bayesian Nonparametric Autoregressive Hidden Markov Models",Hongmin Wu, Yisheng Guan and Juan Rojas, International Journal of Advanced Robotics Systems, 2018 (IF 0.952).[http://www.juanrojas.net/files/papers/2019IJARS-Wu_Rojas-AnalysisHDPVARHMM.pdf](http://www.juanrojas.net/files/papers/2019IJARS-Wu_Rojas-AnalysisHDPVARHMM.pdf)
   
