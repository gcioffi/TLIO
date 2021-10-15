'''
This script rotates real imu measurements to body frame of simulated imu.

Reference frames:
- S: simulated imu frame 
- R: real imu frame 

Notation:
- T: 6DoF transformation matrix. Pose class
- R: rotation matrix. (3,3) numpy vector  
- p: position vector. (3,) numpy vector 
- t: time. Scalar

Inputs:
- real_imu_fn.txt : Real imu measurements
- sim_imu_fn.txt : Simulated imu measurements
- toffset: treal = tsim + toffset. We found that 32.9 s is a good guess.
- theta: yaw rotation offset between real and sim imu frames. Use the plot before alignment to estimate it.
We found that 100 deg is a good guess.

For the format see: https://github.com/CathIAS/TLIO (File formats, imu_measurements.txt)

Outputs:
- aligned_real_imu_measurements.txt : Real imu measurements written in frame S

'''


import argparse
import os

import IPython
import matplotlib.pyplot as plt
import numpy as np
from pyquaternion import Quaternion

import alignment_utils
import pose


def plotImu(real_imu, sim_imu):
	real_imu_t = real_imu[0]
	real_imu_acc = real_imu[1]
	real_imu_gyro = real_imu[2]

	sim_imu_t = sim_imu[0]
	sim_imu_acc = sim_imu[1]
	sim_imu_gyro = sim_imu[2]

	# Plot accel.
	plt.subplot(321)
	plt.plot(real_imu_t, real_imu_acc[:, 0], label='real')
	plt.plot(sim_imu_t, sim_imu_acc[:, 0], label='sim')
	plt.grid()
	plt.legend()
	plt.xlabel('t')
	plt.ylabel('accel. x')
	plt.title('Linear accelerations')

	plt.subplot(323)
	plt.plot(real_imu_t, real_imu_acc[:, 1], label='real')
	plt.plot(sim_imu_t, sim_imu_acc[:, 1], label='sim')
	plt.grid()
	plt.legend()
	plt.xlabel('t')
	plt.ylabel('accel. y')

	plt.subplot(325)
	plt.plot(real_imu_t, real_imu_acc[:, 2], label='real')
	plt.plot(sim_imu_t, sim_imu_acc[:, 2], label='sim')
	plt.grid()
	plt.legend()
	plt.xlabel('t')
	plt.ylabel('accel. z')

	# Plot gyro.
	plt.subplot(322)
	plt.plot(real_imu_t, real_imu_gyro[:, 0], label='real')
	plt.plot(sim_imu_t, sim_imu_gyro[:, 0], label='sim')
	plt.grid()
	plt.legend()
	plt.xlabel('t')
	plt.ylabel('gyro. x')
	plt.title('Angular velocities')

	plt.subplot(324)
	plt.plot(real_imu_t, real_imu_gyro[:, 1], label='real')
	plt.plot(sim_imu_t, sim_imu_gyro[:, 1], label='sim')
	plt.grid()
	plt.legend()
	plt.xlabel('t')
	plt.ylabel('gyro. y')

	plt.subplot(326)
	plt.plot(real_imu_t, real_imu_gyro[:, 2], label='real')
	plt.plot(sim_imu_t, sim_imu_gyro[:, 2], label='sim')
	plt.grid()
	plt.legend()
	plt.xlabel('t')
	plt.ylabel('gyro. z')


def align(t_A, p_AP, t_B, p_BP):
	# Downsample to 20 Hz (otherwise, index search'll be very slow)
	dt = 0.050

	if ( (t_A[1] - t_A[0]) < (dt - 0.001) ):
		t_k = t_A[0]
		t_sampled = []
		p_sampled = []
		t_sampled.append(t_k)
		p_sampled.append(p_AP[0])
		for i, t in enumerate(t_A):
			if t >= (dt + t_k):
				t_k = t
				t_sampled.append(t_k)
				p_sampled.append(p_AP[i])
		t_A = np.asarray(t_sampled)
		p_AP = np.asarray(p_sampled)

	if ( (t_B[1] - t_B[0]) < (dt - 0.001) ):
		t_k = t_B[0]
		t_sampled = []
		p_sampled = []
		t_sampled.append(t_k)
		p_sampled.append(p_BP[0])
		for i, t in enumerate(t_B):
			if t >= (dt + t_k):
				t_k = t
				t_sampled.append(t_k)
				p_sampled.append(p_BP[i])
		t_B = np.asarray(t_sampled)
		p_BP = np.asarray(p_sampled)

	idx_t_matches = alignment_utils.associateTimestamps(t_A, t_B)
	idx_t_A = [idx[0] for idx in idx_t_matches]
	idx_t_B = [idx[1] for idx in idx_t_matches]

	p_AP_to_align = p_AP[idx_t_A]
	p_BP_to_align = p_BP[idx_t_B]

	assert p_AP_to_align.shape == p_BP_to_align.shape

	_, R_AB, p_AB = alignment_utils.alignUmeyama(p_AP_to_align, p_BP_to_align, True, True)
	T_AB = pose.Pose(R_AB, p_AB.reshape(3, 1))
	return T_AB


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("--real_imu_fn", type=str)
    parser.add_argument("--sim_imu_fn", type=str)
    parser.add_argument("--toffset", type=float) # [us]
    parser.add_argument("--theta", type=float) # [deg]
    args = parser.parse_args()

    theta_deg = args.theta
    theta_rad = np.deg2rad(theta_deg)
    R_SR = np.array([
    	[np.cos(theta_rad), -np.sin(theta_rad), 0.0],
    	[np.sin(theta_rad), np.cos(theta_rad), 0.0], 
    	[0.0, 0.0, 1.0]]).T
    
    # read measurements
    real_imu_fn = args.real_imu_fn
    sim_imu_fn = args.sim_imu_fn

    real_imu = np.loadtxt(real_imu_fn)
    sim_imu = np.loadtxt(sim_imu_fn)

    real_imu_t = real_imu[:,0]
    real_imu_t -= real_imu_t[0]
    real_imu_acc_raw = real_imu[:,1:4]
    real_imu_acc_calib = real_imu[:,4:7]
    real_imu_gyro_raw = real_imu[:,7:10]
    real_imu_gyro_calib = real_imu[:,10:13]

    sim_imu_t = sim_imu[:,0]
    sim_imu_t -= sim_imu_t[0]
    sim_imu_acc_raw = sim_imu[:,1:4]
    sim_imu_acc_calib = sim_imu[:,4:7]
    sim_imu_gyro_raw = sim_imu[:,7:10]
    sim_imu_gyro_calib = sim_imu[:,10:13]

    # real_imu_t = sim_imu_t + t_offset_sim_real
    t_offset_sim_real = args.toffset # [us]
    print('==== Using t_offset_sim_real = %.1f ====' % t_offset_sim_real)
    real_imu_t -= t_offset_sim_real

    real_imu_t = np.array([t for t in real_imu_t if t > 0])
    N = real_imu_t.shape[0]
    real_imu_acc_raw = real_imu_acc_raw[-N:]
    real_imu_acc_calib = real_imu_acc_calib[-N:]
    real_imu_gyro_raw = real_imu_gyro_raw[-N:]
    real_imu_gyro_calib = real_imu_gyro_calib[-N:]
    
    # Visualize
    title = 'IMU calib before alignment'
    plt.figure(title)
    plotImu([real_imu_t, real_imu_acc_calib, real_imu_gyro_calib], 
    	[sim_imu_t, sim_imu_acc_calib, sim_imu_gyro_calib])

    # Align
    aligned_real_imu_acc_calib = np.array([np.dot(R_SR, p) for p in real_imu_acc_calib])
    aligned_real_imu_gyro_calib = np.array([np.dot(R_SR, p) for p in real_imu_gyro_calib])

    # Visualize
    title = 'IMU calib after alignment'
    plt.figure(title)
    plotImu([real_imu_t, aligned_real_imu_acc_calib, aligned_real_imu_gyro_calib], 
    	[sim_imu_t, sim_imu_acc_calib, sim_imu_gyro_calib])
    plt.show()

