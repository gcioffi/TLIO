'''
This scripts plots position and orientation of real and simulated poses.
It takes as input the time offset to help to initially manually align the signals

Inputs:
- real_csv: .csv containing real poses
- sim_csv: .csv containing sim poses
- toffset: treal = tsim + toffset
- save: save the time aligned poses

Output:
- given the argument --save, time aligned poses are saved in .csv files ready to be use in hand-eye calibration
'''

import argparse
import os

import csv
import matplotlib.pyplot as plt
import numpy as np
from pyquaternion import Quaternion

import pose
import transformations as tf


if __name__ == '__main__':
	parser = argparse.ArgumentParser()
	parser.add_argument("--real_csv", type=str)
	parser.add_argument("--sim_csv", type=str)
	parser.add_argument("--toffset", type=float)
	parser.add_argument('--save', dest='save', action='store_true')
	parser.add_argument('--no-save', dest='save', action='store_false')
	parser.set_defaults(save=False)
	args = parser.parse_args()

	# read data
	with open(args.real_csv, 'r') as csvfile:
		poses_reader = csv.reader(csvfile, delimiter=',', quotechar='|')
		poses_array = np.array(list(poses_reader))
		real_poses = poses_array.astype(float)

	with open(args.sim_csv, 'r') as csvfile:
		poses_reader = csv.reader(csvfile, delimiter=',', quotechar='|')
		poses_array = np.array(list(poses_reader))
		sim_poses = poses_array.astype(float)


	# read rotations in euler representation
	real_ori_zyx = []
	for p in real_poses:
		R = Quaternion(np.array([p[7], p[4], p[5], p[6]])).rotation_matrix
		rz, ry, rx = tf.euler_from_matrix(R, 'rzyx')
		real_ori_zyx.append(np.array([rz, ry, rx]))
	real_ori_zyx = np.asarray(real_ori_zyx)
	real_ori_zyx = np.rad2deg(real_ori_zyx)

	sim_ori_zyx = []
	for p in sim_poses:
		R = Quaternion(np.array([p[7], p[4], p[5], p[6]])).rotation_matrix
		rz, ry, rx = tf.euler_from_matrix(R, 'rzyx')
		sim_ori_zyx.append(np.array([rz, ry, rx]))
	sim_ori_zyx = np.asarray(sim_ori_zyx)
	sim_ori_zyx = np.rad2deg(sim_ori_zyx)

	t_offset = args.toffset
	print('Using time offset = %.3f' % t_offset)
	real_poses[:,0] -= t_offset

	# Plotting
	plt.figure('Position')
	plt.subplot(311)
	plt.plot(real_poses[:,0], real_poses[:,1], label='real')
	plt.plot(sim_poses[:,0], sim_poses[:,1], label='sim')
	plt.grid()
	plt.legend()
	plt.xlabel('t')
	plt.ylabel('x')

	plt.subplot(312)
	plt.plot(real_poses[:,0], real_poses[:,2], label='real')
	plt.plot(sim_poses[:,0], sim_poses[:,2], label='sim')
	plt.grid()
	plt.legend()
	plt.xlabel('t')
	plt.ylabel('y')

	plt.subplot(313)
	plt.plot(real_poses[:,0], real_poses[:,3], label='real')
	plt.plot(sim_poses[:,0], sim_poses[:,3], label='sim')
	plt.grid()
	plt.legend()
	plt.xlabel('t')
	plt.ylabel('z')

	plt.figure('Orientations')
	plt.subplot(311)
	plt.plot(real_poses[:,0], real_ori_zyx[:, 0], label='real')
	plt.plot(sim_poses[:,0], sim_ori_zyx[:, 0], label='sim')
	plt.grid()
	plt.legend()
	plt.xlabel('t')
	plt.ylabel('yaw')

	plt.subplot(312)
	plt.plot(real_poses[:,0], real_ori_zyx[:, 1], label='real')
	plt.plot(sim_poses[:,0], sim_ori_zyx[:, 1], label='sim')
	plt.grid()
	plt.legend()
	plt.xlabel('t')
	plt.ylabel('pitch')

	plt.subplot(313)
	plt.plot(real_poses[:,0], real_ori_zyx[:, 2], label='real')
	plt.plot(sim_poses[:,0], sim_ori_zyx[:, 2], label='sim')
	plt.grid()
	plt.legend()
	plt.xlabel('t')
	plt.ylabel('roll')

	plt.show()

	if args.save:
		if args.real_csv[:12] == 'time_aligned': 
			print('I am not gonna save. Inputs are time aligned poses.')

		else:
			real_csvfn = os.path.join(os.path.dirname(args.real_csv), 'time_aligned_' + 
				os.path.basename(args.real_csv))
			real_csv_file = open(real_csvfn, 'w')
			for s in real_poses:
				if s[0] >= 0.0:
					real_csv_file.write(str(s[0]) + ', ' + 
						str(s[1]) + ', ' +  str(s[2]) + ', ' + str(s[3]) + ', ' + 
						str(s[4]) + ', ' + str(s[5]) + ', ' + str(s[6]) + ', ' + str(s[7]) + '\n')
			real_csv_file.close()

			sim_csvfn = os.path.join(os.path.dirname(args.sim_csv), 'time_aligned_' + 
				os.path.basename(args.sim_csv))
			sim_csv_file = open(sim_csvfn, 'w')
			for s in sim_poses:
				if s[0] >= 0.0:
					sim_csv_file.write(str(s[0]) + ', ' + 
						str(s[1]) + ', ' +  str(s[2]) + ', ' + str(s[3]) + ', ' + 
						str(s[4]) + ', ' + str(s[5]) + ', ' + str(s[6]) + ', ' + str(s[7]) + '\n')
			sim_csv_file.close()

