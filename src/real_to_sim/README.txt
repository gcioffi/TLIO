# Problem

Estimate the relative rotation between the real and the simulated imu.

# Solution

- Transform the Vicon pose measurements from the center of the markers to the imu frame of the sevensense camera.
-- Run handeye (https://github.com/ethz-asl/hand_eye_calibration/tree/master) to estimate the 6 DoF relative transformation between the Vicon markers (= hand) and the camera (= eye). See "How to run ASL handeye" for more details.
-- Use the script: from_vicon_to_imu.py to get real_body_evolving_state.txt which contains the poses and velocity of the imu frame of the sevensense camera.
WARNING: This script contains hand-coded values (handeye matrix and cam-imu transformation).

- Estimate the rotation offset between the real and simulated imu.
-- Use the script compare_sim_and_real_trajs.py to plot sim and real trajectories before and after rotation alignment.
This script takes as input the angle **theta** which is the rotation offset along the gravity between real and sim. It uses this theta to rotate the real imu to the sim imu frame. Use the plots before alignment to estimate **theta**.

- Align the real imu measurements to the simulated measurements
-- Use the script align_imu_real_to_sim.py with theta estimated at the previous step. This will save a new .txt containing the real imu measurements aligned to the sim ones.


## How to run ASL handeye
To Do :) 