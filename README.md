### Acquire the real-flight rosbag

Acquire the original rosbag during a real flown trajectory, recording the topics:     

- /alphasense_driver_ros/cam0 : sensor_msgs/Image        
- /alphasense_driver_ros/cam1 : sensor_msgs/Image        
- /alphasense_driver_ros/imu  : sensor_msgs/Imu          
- /vicon/parrot   

### Adjust the real rosbag

Use "Rosbag_Add_reference_frame.py" in src/scripts to add the right frame needed by RVIZ for the visualization of the original bag. In RVIZ, under "fixed frame", type "vicon".

Play the original bag on your lapotp and record a copy of it, starting from the moment in which the drone is already in the starting position and trying to include a few seconds of hover in the copy bag. Stop the copy bag when the drone is in its final position, removing the landing part present in the original rosbag. 

From now on, I will refer to the copy bag as real-flight bag. 

### Load the trajectory 

Use "load_real_flight_bag.py" to load the real-flight bag.
The following files will be generated:

- my_timestamps_p.txt (measurements at 400 Hz)
- imu_measurements.txt (measurements at 800 Hz)
- evolving_state.txt (measurements at 400 Hz)

In "evolving_state.txt", the VICON velocity will not be inserted for the moment and VICON/IMU are not time-synchronized. Thus, it is necessary to execute all the next steps in order to have a complete version of "evolving_state.txt". 

In src/params/dataloader_params.yaml change bagfile and out_dir, according to the directory of your copy bag. 

**Command to launch**

python3 src/dataloader/load_real_flight_bag.py --config src/params/dataloader_params.yaml

### Synchronize Vicon - IMU

Go to the src/Vicon/data folder and copy your bag here. Then, in src/Vicon/src/main.py, line 318, insert the bag name without the extension '.bag'.

Now, running "Sync_Vicon_IMU.py", a csv file will be output in src/Vicon/data. 
After that, run the Vicon main to get time offset between the Vicon and the Imu in the real trajectory. 
Remember: t_sync_vicon = t_vicon - offset.

Go to src/scripts and type:

**Command to launch**

python3 Sync_Vicon_IMU.py

Go to src/Vicon/src and type:

**Command to launch**

python main.py

### Load the synchronized trajectory 

Subtract the offset value obtained before from ts_odom in "load_real_flight_bag.py_sync.py" (see lines 137 and 148) and run this script to load again the files loaded before but now synchronized. 

**Command to launch**

python3 src/dataloader/load_real_flight_bag_sync.py --config src/params/dataloader_params.yaml

### Load the simulated trajectory 

Change branch, go to "simulate_real_noise" and run "load_random_trajectories_from_rosbag", loading the simulated bag of the real-flown trajectory. In this way, we can then compare the simulated vs. real trajectory tests of the traind model.  

### Add VICON velocity and plot

Use "Replace_Evolving_State_and_Plot.py" in src/scripts in order to read the VICON computed velocity (from VICON folder - Christian Pfeiffer) and timestamps and insert the velocity values in the evolving_state.txt.
In addition, some plots about position and velocity GT vs. VICON will be generated.

Just make sure that the directories in this script correspond to the directories in your workspace. It is also crucial to insert the time offset between the simulated and the real trajectory in the plotting part, in order to have them aligned. 
Go to src/scripts and type: 

**Command to launch**

python3 Replace_Evolving_State_and_Plot.py

### Interpolate data at the required frequency

Use "Interpolate_Higher_Frequency.py" to get the data at the TLIO required frequency. 
Just make sure that the directories in this script correspond to the directories in your workspace. 

### Plot IMU Vicon and IMU simulated 

Use "Plot_IMU_Vicon_vs_GT.py" to plot the IMU from the VICON and the IMU from the simulated bag. 
Just make sure that the directories in this script correspond to the directories in your workspace and that you insert the time offset between simulated and real trajectory found in the previous step.

### Modify HasVIO vector in evolving_state.txt

Use "Transform_HasVio.py" to get more correspondences between IMU states and corresponding VIO states. 
Just make sure that the directories in this script correspond to the directories in your workspace.

### Generate hdf5

Launching "gen_racing_data.py", it is possible to get the hdf5 file needed for the training step and the train.txt, test.txt and val.txt files.
When launching this script, a data directory --data_dir should be specified: TLIO/data/Dataset. 

**Command to launch**

python3 src/dataloader/gen_racing_data.py --data_dir data/...("your_dataset_directory")...

