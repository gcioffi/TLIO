'''
This script shifts the time in real data in ordet to align it to sim data.
See here (https://github.com/gcioffi/TLIO/tree/master) for the format of evolving_state.txt

Inputs:
- real_ev_fn: .txt containing real data
- sim_ev_fn: .txt containing sim data
- toffset: treal = tsim + toffset. We found that 32.9 s is a good guess.
- theta: yaw rotation offset between real and sim imu frames. Use the plot before alignment to estimate it.
We found that 100 deg is a good guess.

Output:
- None
'''

import argparse
import os

import csv
import matplotlib.pyplot as plt
import numpy as np
from pyquaternion import Quaternion

import alignment_utils
import pose
import transformations as tf


def getRotationEulerZYX(poses):
	ori_zyx = []
	for p in poses:
		R = Quaternion(np.array([p[1], p[2], p[3], p[4]])).rotation_matrix
		rz, ry, rx = tf.euler_from_matrix(R, 'rzyx')
		ori_zyx.append(np.array([rz, ry, rx]))
	ori_zyx = np.asarray(ori_zyx)
	ori_zyx = np.rad2deg(ori_zyx)
	return ori_zyx


def plotPosition(real_poses, sim_poses):
	plt.subplot(311)
	plt.plot(real_poses[:,0], real_poses[:,5], label='real')
	plt.plot(sim_poses[:,0], sim_poses[:,5], label='sim')
	plt.grid()
	plt.legend()
	plt.xlabel('t')
	plt.ylabel('x')

	plt.subplot(312)
	plt.plot(real_poses[:,0], real_poses[:,6], label='real')
	plt.plot(sim_poses[:,0], sim_poses[:,6], label='sim')
	plt.grid()
	plt.legend()
	plt.xlabel('t')
	plt.ylabel('y')

	plt.subplot(313)
	plt.plot(real_poses[:,0], real_poses[:,7], label='real')
	plt.plot(sim_poses[:,0], sim_poses[:,7], label='sim')
	plt.grid()
	plt.legend()
	plt.xlabel('t')
	plt.ylabel('z')

	plt.show()


def plotOrientation(t_real, real_ori_zyx, t_sim, sim_ori_zyx):
	plt.subplot(311)
	plt.plot(t_real, real_ori_zyx[:, 0], label='real')
	plt.plot(t_sim, sim_ori_zyx[:, 0], label='sim')
	plt.grid()
	plt.legend()
	plt.xlabel('t')
	plt.ylabel('yaw')

	plt.subplot(312)
	plt.plot(t_real, real_ori_zyx[:, 1], label='real')
	plt.plot(t_sim, sim_ori_zyx[:, 1], label='sim')
	plt.grid()
	plt.legend()
	plt.xlabel('t')
	plt.ylabel('pitch')

	plt.subplot(313)
	plt.plot(t_real, real_ori_zyx[:, 2], label='real')
	plt.plot(t_sim, sim_ori_zyx[:, 2], label='sim')
	plt.grid()
	plt.legend()
	plt.xlabel('t')
	plt.ylabel('roll')

	plt.show()
	
def matchTimes(t_A, t_B):
	# Downsample to 20 Hz (otherwise, index search'll be very slow)
	dt = 0.050

	if ( (t_A[1] - t_A[0]) < (dt - 0.001) ):
		t_k = t_A[0]
		t_sampled = []
		t_sampled.append(t_k)
		for i, t in enumerate(t_A):
			if t >= (dt + t_k):
				t_k = t
				t_sampled.append(t_k)
		t_A = np.asarray(t_sampled)

	if ( (t_B[1] - t_B[0]) < (dt - 0.001) ):
		t_k = t_B[0]
		t_sampled = []
		t_sampled.append(t_k)
		for i, t in enumerate(t_B):
			if t >= (dt + t_k):
				t_k = t
				t_sampled.append(t_k)
		t_B = np.asarray(t_sampled)

	idx_t_matches = alignment_utils.associateTimestamps(t_A, t_B)
	idx_t_A = [idx[0] for idx in idx_t_matches]
	idx_t_B = [idx[1] for idx in idx_t_matches]

	assert len(idx_t_A) == len(idx_t_B)

	return idx_t_A, idx_t_B


if __name__ == '__main__':
	parser = argparse.ArgumentParser()
	parser.add_argument("--real_ev_fn", type=str)
	parser.add_argument("--sim_ev_fn", type=str)
	parser.add_argument("--toffset", type=float) # [s]
	parser.add_argument("--theta", type=float) # [degree]
	args = parser.parse_args()

	# read data
	real_poses = np.loadtxt(args.real_ev_fn)
	sim_poses = np.loadtxt(args.sim_ev_fn)
	real_poses[:, 0] -= real_poses[0,0]
	sim_poses[:, 0] -= sim_poses[0,0]
	# read rotations in euler representation
	real_ori_zyx = getRotationEulerZYX(real_poses)
	sim_ori_zyx = getRotationEulerZYX(sim_poses)

	# Apply rotation
	theta_deg = args.theta
	theta_rad = np.deg2rad(theta_deg)

	dR = np.array([
		[np.cos(theta_rad), -np.sin(theta_rad), 0.0],
		[np.sin(theta_rad), np.cos(theta_rad), 0.0],
		[0.0, 0.0, 1.0]])
	rotated_poses = []
	for p in real_poses:
		R = Quaternion(np.array([p[1], p[2], p[3], p[4]])).rotation_matrix
		R_new = np.dot(R, dR)
		q_new = Quaternion(matrix=R_new)
		rotated_poses.append(np.array([p[0],
			q_new[0], q_new[1], q_new[2], q_new[3], 
			p[5], p[6], p[7], p[8], p[9], p[10]]))
	real_poses = np.asarray(rotated_poses)

	real_ori_zyx = getRotationEulerZYX(real_poses)

	plt.figure('Position after time alignment')
	plotPosition(real_poses, sim_poses)


	plt.figure('Orientations after time alignment')
	plotOrientation(real_poses[:,0], real_ori_zyx, sim_poses[:,0], sim_ori_zyx)

	plt.show()

	# This was used for some tests with handeye
	# match times
	'''idx_t_real, idx_t_sim = matchTimes(real_poses[:, 0], sim_poses[:, 0])
	indices = np.hstack((np.asarray(idx_t_real).reshape(-1,1), 
		np.asarray(idx_t_sim).reshape(-1,1)))

	matched_real_poses = real_poses[idx_t_real]
	matched_sim_poses = sim_poses[idx_t_sim]'''

	# save
	'''outfn = os.path.join(os.path.dirname(args.real_ev_fn), 'aligned_' + 
				os.path.basename(args.real_ev_fn))
	np.savetxt(outfn, real_poses)

	outfn = os.path.join(os.path.dirname(args.real_ev_fn), 'matched_' + 
		os.path.basename(args.real_ev_fn))
	np.savetxt(outfn, matched_real_poses)
			
	outfn = os.path.join(os.path.dirname(args.sim_ev_fn), 'matched_' + 
				os.path.basename(args.sim_ev_fn))
	np.savetxt(outfn, matched_sim_poses)

	outfn = os.path.join(os.path.dirname(args.real_ev_fn), 'index_matches.txt')
	np.savetxt(outfn, indices, fmt='%d')'''

