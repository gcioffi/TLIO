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
    parser.add_argument("--toffset", type=float) # [s]
    parser.add_argument("--theta", type=float) # [deg]
    args = parser.parse_args()

    # @TODO: load from args.handeye
    q_MC = Quaternion(np.array([0.90313326730181243, 
    	-0.010070363126693096, 
    	0.42923296948858486,
    	-0.0028190748946359361]))
    R_MC = q_MC.rotation_matrix
    p_MC = np.array([0.070870689700682124, 
		-0.019621944091964213, 
		0.042258246141882193])
    T_MC = pose.Pose(R_MC, p_MC.reshape((3,1)))
    # t_c = t_m + t_offset_mc (see: https://github.com/ethz-asl/hand_eye_calibration/blob/966cd92518f24aa7dfdacc8ba9c5fa4a441270cd/hand_eye_calibration/python/hand_eye_calibration/time_alignment.py#L56)
    t_offset_mc = -0.0084234129999999994

    # @TODO: load from args.camimu_calib
    R_CB = np.array([[0.00629031, 0.62608545, -0.77972908], 
    	[-0.99997865, 0.00255932, -0.00601211], 
    	[-0.00176851, 0.77975026, 0.62608818]])
    p_CB = np.array([-0.03474805, 0.00664268, -0.04902435])
    T_CB = pose.Pose(R_CB, p_CB.reshape((3,1)))
    # t_b = t_c + t_offset_cb
    t_offset_cb = 0.0018521255066710702
    
    # t_b = t_m + t_offset_mb
    t_offset_mb = t_offset_mc + t_offset_cb

    # transform poses
    T_MB = T_MC * T_CB
    R_SR = T_MB.R

    '''theta_deg = args.theta
                theta_rad = np.deg2rad(theta_deg)
                R_SR = np.array([
                	[np.cos(theta_rad), -np.sin(theta_rad), 0.0],
                	[np.sin(theta_rad), np.cos(theta_rad), 0.0], 
                	[0.0, 0.0, 1.0]]).T'''
    
    # read measurements
    real_imu_fn = args.real_imu_fn
    sim_imu_fn = args.sim_imu_fn

    real_imu = np.loadtxt(real_imu_fn)
    sim_imu = np.loadtxt(sim_imu_fn)

    real_imu_t = real_imu[:,0]
    #real_imu_t -= real_imu_t[0] 
    #real_imu_t += 266.675000

    sim_imu_t = sim_imu[:,0]

    # real_imu_t = sim_imu_t + t_offset_sim_real
    t_offset_sim_real = args.toffset # [s]
    #t_offset_sim_real = - 0.5
    print('==== Using t_offset_sim_real = %.1f ====' % t_offset_sim_real)
    real_imu_t -= t_offset_sim_real

    real_imu_t = np.array([t for t in real_imu_t if t > 0])
    N = real_imu_t.shape[0]
    real_imu = real_imu[(real_imu.shape[0]-N):]

    real_imu[:,0] = real_imu_t
    
    # Visualize
    title = 'IMU calib before alignment'
    plt.figure(title)
    plotImu([real_imu_t, real_imu[:, 4:], real_imu[:,1:4]], 
    	[sim_imu_t, sim_imu[:, 4:], sim_imu[:,1:4]])

    # Align
    aligned_real_imu_acc = np.array([np.dot(R_SR, p) for p in real_imu[:, 4:]])
    aligned_real_imu_gyro = np.array([np.dot(R_SR, p) for p in real_imu[:, 1:4]])

    # Visualize
    title = 'IMU calib after alignment'
    plt.figure(title)
    plotImu([real_imu_t, aligned_real_imu_acc, aligned_real_imu_gyro], 
    	[sim_imu_t, sim_imu[:, 4:], sim_imu[:, 1:4]])
    plt.show()

    # save
    '''out_dir = '/home/giovanni/TLIO/data/tracking_arena_data/29July21/tracking_arena_2021-02-03-13-43-38/real_markers'
    traj = np.loadtxt(out_dir + '/trajectory.txt')
    vel = np.loadtxt(out_dir + '/velocity.txt') 

    traj[:,0] = real_imu_t
    vel[:,0] = real_imu_t

    np.savetxt(out_dir + '/trajectory.txt', traj, fmt='%.6f')
    np.savetxt(out_dir + '/velocity.txt', vel, fmt='%.6f')
    np.savetxt(out_dir + '/filtered_filtered_2021-02-03-13-43-38_imu_meas.txt', real_imu, fmt='%.6f')'''


    # debug
    '''traj_I = np.loadtxt('/home/giovanni/TLIO/data/tracking_arena_data/29July21/tracking_arena_2021-02-03-13-43-38/real_imu/trajectory.txt')
    traj_B = np.loadtxt('/home/giovanni/gvi-fusion/results/imu_simulator/tracking_arena_2021-02-03-13-43-38/sim_imu_from_vicon_traj/trajectory.txt')

    aligned_real_imu_acc_debug = []
    I_r_IB = -1.0 * np.dot(T_MB.R.T, T_MB.t.flatten())
    for i, I_a in enumerate(real_imu[:,4:]):
    	pose_i = traj_I[i]
    	assert np.abs(pose_i[0] - real_imu[i,0]) < 0.0001, 'pose_i[0] = %.6f, real_imu[i,0] = %.6f' % (pose_i[0], real_imu[i,0])
    	R_WI = Quaternion(np.array([pose_i[7], pose_i[4], pose_i[5], pose_i[6]])).rotation_matrix
    	W_a = np.dot(R_WI, I_a)
    	I_w = real_imu[i, 1:4]
    	W_w = np.dot(R_WI, I_w)
    	W_r_IB = np.dot(R_WI, I_r_IB)

    	W_a_B = W_a + np.cross(W_w, np.cross(W_w, W_r_IB))
    	assert len(W_a_B.shape) == 1
    	assert W_a_B.shape[0] == 3

    	pose_b = traj_B[i]
    	#assert np.abs(pose_b[0] - real_imu[i,0]) < 0.0001, 'pose_b[0] = %.6f, real_imu[i,0] = %.6f' % (pose_b[0], real_imu[i,0])
    	R_WB = Quaternion(np.array([pose_b[7], pose_b[4], pose_b[5], pose_b[6]])).rotation_matrix
    	B_a = np.dot(R_WB.T, W_a_B)

    	aligned_real_imu_acc_debug.append(B_a)
    aligned_real_imu_acc_debug = np.asarray(aligned_real_imu_acc_debug)


    title = 'Debug IMU calib after alignment'
    plt.figure(title)
    plotImu([real_imu_t, aligned_real_imu_acc_debug, aligned_real_imu_gyro], 
    	[sim_imu_t, sim_imu[:, 4:], sim_imu[:, 1:4]])
   	plt.show()'''
    # end

