## TLIO Readme
Plase, find the TLIO Readme at: https://github.com/CathIAS/TLIO/blob/master/README.md


## SNAGA: Introduction

**Useful links**
- Machine learning hardware
https://app.gitbook.com/@rpg-uzh/s/rpg-uzh/computers-storage-and-printers/machine-learning-computer
- Work on SNAGA
https://app.gitbook.com/@rpg-uzh/s/rpg-uzh/computers-storage-and-printers/workshop-pcs-and-snaga
- Work on SNAGA from HOME
https://app.gitbook.com/@rpg-uzh/s/rpg-uzh/computers-storage-and-printers/workshop-pcs-and-snaga

**Connect to SNAGA**

- ssh user@snaga.ifi.uzh.ch

**Create a virtual environment on SNAGA**

- conda deactivate
- conda activate
- conda create --name TLIO
- conda activate TLIO

**Create a directory to store results and files**

In this case, it is:
- /data/storage/user/TLIO
Never store results in the HOME directory! 

**Create soft link between TLIO code in HOME and your results folder**

in ~TLIO: ln -s /data/storage/user/TLIO/data .
in ~TLIO: ln -s /data/storage/user/TLIO/results/ .

**Source ROS**

Add it to BASHRC in order to have it automatically
- source /opt/ros/melodic/setup.bash

**Create a COPY from local laptop to snaga**

Run the following on your local terminal:
- scp -r file user@snaga.ifi.uzh.ch:/data/storage/user/TLIO/data 


**Create a COPY from snaga to local laptop**

- scp -r user@snaga.ifi.uzh.ch:/home/user/TLIO/results/file . 

**Check GPU usage and set a gpu for training**

- gpustat
- export CUDA_VISIBLE_DEVICES=X ---> where X is the number of a free gpu
- screen 
- press enter
"Screen" is needed to avoid killing the script if and when connection is lost.

**Exit SCREEN and SNAGA**

- exit


## Generate Dataset
This could be generated using gvi as explained in the next section (recomended option) or extracting the TLIO needed files using the information from rosbags recorded during flights simulated in Gazebo, namely Agiros and the realted reference trajectory in .csv. 
The files used for TLIO are:

- my_timestamps_p.txt
- imu_measurements.txt
- evolving_state.txt

## Generate Dataset: GVI Sim-to-Real
Here, we describe how to generate a new dataset starting from a .rosbag recorded in the Flying-Arena.
We assume that the .rosbag contains vicon and alphasense data.

Example output of rosbag info:

topics:      
/alphasense_driver_ros/cam0    1074 msgs    : sensor_msgs/Image        
/alphasense_driver_ros/cam1    1074 msgs    : sensor_msgs/Image        
/alphasense_driver_ros/imu    14285 msgs    : sensor_msgs/Imu          
/vicon/parrot                 14308 msgs    : geometry_msgs/PoseStamped

### Handeye

The first step is to get the IMU poses from the vicon trajectory.
We run the [hand-eye](https://github.com/ethz-asl/hand_eye_calibration) to get the camera-markers time offset and relative transformation.
Check src/sim_to_real/scripts for some useful scripts.
Then, run the script sim_to_real/scripts/vicon_to_imu.py [WARNING: this scripts contains hand-coded cam-imu (from Kalibr) and cam-vicon (from handeye) calibrations] to transform (temporally and spatially) the vicon poses from the markers to the imu frame.


## IMU Simulator

We are now ready to generate simulated IMU measurements starting from the real flown trajectory [@ToDo: Extend to simulated trajectory].
We use the code [here](https://github.com/uzh-rpg/gvi-fusion/tree/sim_imu).

To fit a Bspline to the desired trajectory

```./fit_trajectory path-to-config```

For example

```./fit_trajectory ../experiments/imu_simulator/tracking_arena_2021-02-03-13-43-38.yaml```

To check that the fitting has been successful

```python src/sim_to_real/scripts/plot_sim_and_real_trajectories.py --real_fn path-to-txt --sim_fn path-to-txt```

For example

```python src/sim_to_real/scripts/plot_sim_and_real_trajectories.py --real_fn 2021-02-03-13-43-38_traj_vicon_imu.txt --sim_fn trajectory.txt```

To simulate IMU measurements:

```./simulate_imu path-to-config```

For example

```./simulate_imu ../experiments/imu_simulator/tracking_arena_2021-02-03-13-43-38.yaml```

An example of config file is [here](https://github.com/uzh-rpg/gvi-fusion/blob/sim_imu/experiments/imu_simulator/tracking_arena_2021-02-03-13-43-38.yaml)

Note that the current implementation requires that the spline order is given at compilation time. Check [here](https://github.com/uzh-rpg/gvi-fusion/blob/sim_imu/src/imu_simulator/fit_trajectory.cpp#L64) and [here](https://github.com/uzh-rpg/gvi-fusion/blob/sim_imu/src/imu_simulator/simulate_imu.cpp#L89).


## Create Dataset

We low-pass filter real and sim IMU to remove noise coming from the platform (e.g.motors).

To low-pass using a butterworth filter

```python src/sim_to_real/scripts/apply_butterworth_filter.py --config path-to-config```

or

```python src/sim_to_real/scripts/apply_butterworth_filter.py --signal_fn path-to-txt --freq freq --cutoff_freq 50 --config '' ```

For example

This will run on all the seq. specified in the config file. This is the command for creating a new dataset.

```python src/sim_to_real/scripts/apply_butterworth_filter.py --config ./config/sim_to_real/dataset_tracking_arena_2021-02-03-13-43-38.yaml```

or

this will run on the single data contained in the .txt file

```python src/sim_to_real/scripts/apply_butterworth_filter.py --signal_fn 2021-02-03-13-43-38_imu_meas.txt --freq 400```

After filtering, we check that the sim and real IMU match using

```python src/sim_to_real/scripts/plot_sim_and_real_imu.py --real_imu_fn path-to-txt --sim_imu_fn path-to-txt```

and

```python src/sim_to_real/scripts/plot_fft_sim_and_real_imu.py --real_imu_fn path-to-txt --sim_imu_fn path-to-txt```

To create the dataset run the following scripts

```python src/dataloader/create_dataset_txt_format.py --config path-to-config```

```python src/dataloader/create_dataset_binary_format.py --config path-to-config```

For example 

```python src/dataloader/create_dataset_txt_format.py --config ../../config/sim_to_real/dataset_tracking_arena_2021-02-03-13-43-38.yaml```

```python src/dataloader/create_dataset_binary_format.py --config ../../config/sim_to_real/dataset_tracking_arena_2021-02-03-13-43-38.yaml```

The first script will take about 10 min for 1000 sequences. The second will take about 30 min for 1000 sequences.


## Training

**Command to launch:**


```python3 src/main_net.py --mode train --root_dir data/folder_data --train_list data/folder_data/train.txt --val_list data/folder_data/val.txt --out_dir results/folder_results --imu_freq 1000 --imu_base_freq 1000 --window_time 0.5 --epochs 100 --batch_size 64```


## Test using AGIROS

### Acquire the real-flight rosbag

Acquire the original rosbag during a real flown trajectory, recording the topics:     

- */alphasense_driver_ros/imu  : sensor_msgs/Imu*          
- */vicon/parrot*   

### Load the trajectory 

Use ```load_real_flight_bag.py``` to load the real-flight bag.
The following files will be generated:

- *my_timestamps_p.txt* (measurements at 400 Hz)
- *imu_measurements.txt* (measurements at 800 Hz)
- *evolving_state.txt* (measurements at 400 Hz)

In *evolving_state.txt*, the Vicon velocity will not be inserted for the moment and also VICON/IMU are not time-synchronized. Thus, it is necessary to execute all the next steps in order to have a complete version of *evolving_state.txt*. 

In *src/params/dataloader_params.yaml* change **bagfile** and **out_dir**, according to the directory of your copy bag. 

**Command to launch**

```python3 src/dataloader/load_real_flight_bag.py --config src/params/dataloader_params.yaml```


### Synchronize Vicon - IMU

Go to the *src/Vicon/data* folder and **copy** your bag here. Then, in ```src/Vicon/src/main.py```, *line 321*, insert the bag name without the extension *'.bag'*.

Now, running ```Sync_Vicon_IMU.py```, a .csv file will be output in *src/Vicon/data*.

After that, run the Vicon **main** to get time offset between the Vicon and the Imu in the real trajectory.

Remember: *t_sync_vicon = t_vicon - offset*.


**Command to launch**

Go to *src/scripts* and type:

```python3 Sync_Vicon_IMU.py```

Go to *src/Vicon/src* and type:

```python main.py```

### Load the synchronized trajectory 

Using the offset value obtained, subtract it from *ts_odom* in ```load_real_flight_bag_sync.py``` (see *line 85*) and run this script to load again the same files as before but now synchronized. 

**Command to launch**

```python3 src/dataloader/load_real_flight_bag_sync.py --config src/params/dataloader_params.yaml```


### Add Vicon velocity 

Use ```Replace_Evolving_State_and_Plot.py``` in *src/scripts*, in order to read the Vicon computed velocity in *src/Vicon/data* (referred to the center of the markers) and insert the velocity values in *evolving_state.txt*.
In addition, some plots about position and velocity GT vs. VICON could be generated.

Just make sure that the directories in this script correspond to the directories in your workspace. It is also crucial to insert the **time-offset** between a simulated trajectory in the dataset and the real one in the plotting part to have them aligned. 
 
**Command to launch**

Go to *src/scripts* and type:

```python3 Replace_Evolving_State_and_Plot.py```

### Transform Evovling State: from markers to IMU

The first thing to do is to **transform** the Vicon pose measurements from the center of the markers to the imu-frame of the sevensense camera. 

- Run handeye (https://github.com/ethz-asl/hand_eye_calibration/tree/master) to estimate the 6 DoF relative transformation between the Vicon markers (= hand) and the camera (= eye). See *"How to run ASL handeye"* for more details.

- Use the script ```from_vicon_to_imu.py``` to get *evolving_state.txt* which now contains the poses and velocity of the imu-frame of the sevensense camera.

**WARNING**: This script contains hand-coded values (handeye matrix and cam-imu transformation).

**Command to launch**

In *src/scripts_rotation*

```python3 from_vicon_to_imu.py --ev_state_fn /home/rpg/Desktop/RosbagReal_13_43_38/seq1/evolving_state.txt```


### Cut the data to remove landing and take off

**Command to launch**

In *src/sim_to_real/scripts*

```python3 cut_data.py```

(insert beginning and end-time at line 23).

### Rotate imu measurements

If the dataset has sequences simulated using Agiros, the IMU measurements should be rotated from the real frame to the simulated frame.
This is used to find theta given t_offset.


**Command to launch**

In *src/scripts_rotation*:

```python3 align_imu_real_to_sim.py --real_imu_fn /home/rpg/Desktop/RosbagReal_13_43_38/seq1/imu_measurements.txt --sim_imu_fn /home/rpg/Desktop/RosbagSimulated_13_43_38/seq1/imu_measurements.txt --toffset 0 --theta 100```

### Interpolate data at the required frequency

**Command to launch**

In *src/real_to_sim/scripts*

``` python3 interpolate.py ```


### Modify HasVIO vector in *evolving_state.txt*

Use ```transform_HasVio.py``` to get more correspondences between IMU states and the corresponding VIO states.

**Command to launch**

In *src/sim_to_real/scripts*

``` python3 transform_HasVio.py ```


### Generate hdf5

Launching "gen_racing_data.py", it is possible to get the hdf5 file needed for the training step and the train.txt, test.txt and val.txt files.
When launching this script, a data directory --data_dir should be specified: TLIO/data/Dataset. 

**Command to launch**

```python3 src/dataloader/gen_racing_data.py --data_dir data/folder_data```


### Test through hdf5

**Command to launch**
```python3 src/main_net.py --mode test --root_dir data/folder_data/ --test_list data/folder_data/test.txt --model_path results/folder_results/checkpoints/checkpoint_126.pt --out_dir results/folder_results/results/network/ --save_plot --window_time 0.5 --imu_freq 1000 --imu_base_freq 1000 --sample_freq 20```


## Test using GVI

### Create Test Sequence for real data

The following applies at the moment to the sequence 2021-02-03-13-43-38.
After extracting the imu measurement from the rosbag (see src/sim_to_real/bag_to_imu.py), we need to remove peaks in az.

To do it, run

```python src/sim_to_real/scripts/apply_handcrafted_filter.py --signal_fn path-to-txt --freq 400 --window_len_sec 1.0 --noise_std 0.02```

For example

```python src/sim_to_real/scripts/apply_handcrafted_filter.py --signal_fn ./data/tracking_arena_data/29July21/tracking_arena_2021-02-03-13-43-38/2021-02-03-13-43-38_imu_meas.txt --freq 400 --window_len_sec 1.0 --noise_std 0.02```

Then low-pass using a butterworth filter

```python src/sim_to_real/scripts/apply_butterworth_filter.py --signal_fn path-to-txt --freq 400 --cutoff_freq 50 --config ''```

For example

```python src/sim_to_real/scripts/apply_butterworth_filter.py --signal_fn ./data/tracking_arena_data/29July21/tracking_arena_2021-02-03-13-43-38/filtered_2021-02-03-13-43-38_imu_meas.txt --freq 400 --cutoff_freq 50 --config ''```

We are now ready to write the measurements in the TLIO format.

```python src/dataloader/create_sequence_txt_format.py --config_fn path-to-config```

For example 

```python src/dataloader/create_sequence_txt_format.py --config_fn config/sim_to_real/tracking_arena_2021-02-03-13-43-38.yaml ```

and 

```python src/dataloader/create_sequence_binary_format.py --config_fn path-to-config```

For example 

```python src/dataloader/create_sequence_binary_format.py --config_fn config/sim_to_real/tracking_arena_2021-02-03-13-43-38.yaml ```


## Run Filter and Plot

**Command to launch**

```python3 src/main_filter.py --root_dir data/folder_data --data_list data/folder_data/test.txt --model_path results/folder_results/checkpoints/checkpoint_126.pt --model_param_path results/folder_results/parameters.json --out_dir results/folder_results/results/filter/ --update_freq 20 --initialize_with_offline_calib --erase_old_log```

To plot, go to TLIO/src and launch:

**Command to launch:**

```python3 plot_filter_results.py --data_dir ../data/folder_data --data_list ../data/folder_data/test.txt --filter_dir ../results/folder_results/results/filter/```



