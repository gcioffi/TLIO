### Acquire the real-flight rosbag

Acquire the original rosbag during a real flown trajectory, recording the topics:     

- */alphasense_driver_ros/cam0 : sensor_msgs/Image*        
- */alphasense_driver_ros/cam1 : sensor_msgs/Image*      
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

Go to the *src/Vicon/data* folder and **copy** your bag here. Then, in ```src/Vicon/src/main.py```, *line 320*, insert the bag name without the extension *'.bag'*.

Now, running ```Sync_Vicon_IMU.py```, a .csv file will be output in *src/Vicon/data*.

After that, run the Vicon **main** to get time offset between the Vicon and the Imu in the real trajectory.

Remember: *t_sync_vicon = t_vicon - offset*.


**Command to launch**

Go to *src/scripts* and type:

```python3 Sync_Vicon_IMU.py```

Go to *src/Vicon/src* and type:

```python main.py```

### Load the synchronized trajectory 

Using the offset value obtained, subtract it from *ts_odom* in ```load_real_flight_bag_sync.py``` (see *line 117*) and run this script to load again the same files as before but now synchronized. 

**Command to launch**

```python3 src/dataloader/load_real_flight_bag_sync.py --config src/params/dataloader_params.yaml```

### Load the simulated trajectory 

Change **branch**, go to ```simulate_real_noise```. 

In *src/params/dataloader_params.yaml* change **bagfile** and **out_dir**, according to the directory of your copy-bag. 

Run ```load_random_trajectories_from_rosbag```, loading the simulated bag of the real-flown trajectory. 

In this way, we can then compare the simulated vs. the real trajectory tests of the traind model.  

### Add Vicon velocity and plot

Use ```Replace_Evolving_State_and_Plot.py``` in *src/scripts*, in order to read the Vicon computed velocity in *src/Vicon/data* (referred to the center of the markers) and insert the velocity values in *evolving_state.txt*.
In addition, some plots about position and velocity GT vs. VICON will be generated.

Just make sure that the directories in this script correspond to the directories in your workspace. It is also crucial to insert the **time-offset** between the simulated and the real trajectory in the plotting part, in order to have them aligned. 
 

**Command to launch**

Go to *src/scripts* and type:

```python3 Replace_Evolving_State_and_Plot.py```


### Transform Evovling State: from markers to IMU

The first thing to do is to **transform** the Vicon pose measurements from the center of the markers to the imu-frame of the sevensense camera. 

- Run handeye (https://github.com/ethz-asl/hand_eye_calibration/tree/master) to estimate the 6 DoF relative transformation between the Vicon markers (= hand) and the camera (= eye). See *"How to run ASL handeye"* for more details.

- Use the script ```from_vicon_to_imu.py``` to get *evolving_state.txt* which now contains the poses and velocity of the imu-frame of the sevensense camera.

**WARNING**: This script contains hand-coded values (handeye matrix and cam-imu transformation).

**Command to launch**

In *src/real_to_sim/scripts*

```python3 from_vicon_to_imu.py --ev_state_fn /home/rpg/Desktop/RosbagReal_13_43_38/seq1/evolving_state.txt```


### Cut the data to remove landing and take off


**Command to launch**

In *src/real_to_sim/scripts*

```python3 cut_data.py```


### Generate file needed by the simulator


**Command to launch**

In *src/real_to_sim/scripts*

```python3 generate_traj_vicon_imu.py```


### Generate data from the simulator


**Command to launch**

In *gvi-fusion/build*

``` ./sim_imu_from_bspline ../experiments/sim_imu tracking_arena_2021-02-03-13-43-38.yaml```



### Plot real vs simulated data


**Command to launch**

In *src/real_to_sim/scripts*

``` python3 plot_meas_vs_sim.py ```


### Generate imu_measurements.txt with simulated data


**Command to launch**

In *src/real_to_sim/scripts*

``` python3 write_imu_measurements.py ```


### Interpolate data at the required frequency

**Command to launch**

In *src/real_to_sim/scripts*

``` python3 interpolate.py ```


### Modify HasVIO vector in *evolving_state.txt*

Use ```transform_HasVio.py``` to get more correspondences between IMU states and the corresponding VIO states.

**Command to launch**

In *src/real_to_sim/scripts*

``` python3 transform_HasVio.py ```


### Generate hdf5

Launching "gen_racing_data.py", it is possible to get the hdf5 file needed for the training step and the train.txt, test.txt and val.txt files.
When launching this script, a data directory --data_dir should be specified: TLIO/data/Dataset. 

**Command to launch**

python3 src/dataloader/gen_racing_data.py --data_dir data/...("your_dataset_directory")...

