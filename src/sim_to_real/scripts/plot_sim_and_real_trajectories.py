'''
This script plots sim and real trajectories
See here output of bag_to_pose.py for the format of .txt files 

Inputs:
- real_fn: .txt containing real data
- sim_fn: .txt containing sim data

Output:
- None
'''

import argparse
import os

import csv
import matplotlib.pyplot as plt
import numpy as np
from pyquaternion import Quaternion

import transformations as tf


def getRotationEulerZYX(poses):
	ori_zyx = []
	for p in poses:
		R = Quaternion(np.array([p[7], p[4], p[5], p[6]])).rotation_matrix
		rz, ry, rx = tf.euler_from_matrix(R, 'rzyx')
		ori_zyx.append(np.array([rz, ry, rx]))
	ori_zyx = np.asarray(ori_zyx)
	ori_zyx = np.rad2deg(ori_zyx)
	return ori_zyx


def plotPosition2D(real_pos, sim_pos, labelx, labely, title):
	plt.plot(real_pos[:,0], real_pos[:,1], label='Real')
	plt.plot(sim_pos[:,0], sim_pos[:,1], label='Sim')
	plt.grid()
	plt.legend(fontsize=15)
	plt.title(title, fontsize=20)
	plt.xlabel(labelx, fontsize=15)
	plt.ylabel(labely, fontsize=15)


def plotPosition1D(t_real, real_pos, t_sim, sim_pos, title):
	plt.subplot(311)
	plt.plot(t_real, real_pos[:, 0], label='Real')
	plt.plot(t_sim, sim_pos[:, 0], label='Sim')
	plt.grid()
	plt.legend(fontsize=15)
	plt.title(title, fontsize=20)
	plt.xlabel('t [s]', fontsize=15)
	plt.ylabel('x [m]', fontsize=15)

	plt.subplot(312)
	plt.plot(t_real, real_pos[:, 1], label='Real')
	plt.plot(t_sim, sim_pos[:, 1], label='Sim')
	plt.grid()
	plt.legend(fontsize=15)
	plt.xlabel('t [s]', fontsize=15)
	plt.ylabel('y [m]', fontsize=15)

	plt.subplot(313)
	plt.plot(t_real, real_pos[:, 2], label='Real')
	plt.plot(t_sim, sim_pos[:, 2], label='Sim')
	plt.grid()
	plt.legend(fontsize=15)
	plt.xlabel('t [s]', fontsize=15)
	plt.ylabel('z [m]', fontsize=15)


def plotOrientation(t_real, real_ori_zyx, t_sim, sim_ori_zyx, title):
	plt.subplot(311)
	plt.plot(t_real, real_ori_zyx[:, 0], label='Real')
	plt.plot(t_sim, sim_ori_zyx[:, 0], label='Sim')
	plt.grid()
	plt.title(title, fontsize=20)
	plt.legend(fontsize=15)
	plt.xlabel('t [s]', fontsize=15)
	plt.ylabel('yaw [°]', fontsize=15)

	plt.subplot(312)
	plt.plot(t_real, real_ori_zyx[:, 1], label='Real')
	plt.plot(t_sim, sim_ori_zyx[:, 1], label='Sim')
	plt.grid()
	plt.legend(fontsize=15)
	plt.xlabel('t [s]', fontsize=15)
	plt.ylabel('pitch [°]', fontsize=15)

	plt.subplot(313)
	plt.plot(t_real, real_ori_zyx[:, 2], label='Real')
	plt.plot(t_sim, sim_ori_zyx[:, 2], label='Sim')
	plt.grid()
	plt.legend(fontsize=15)
	plt.xlabel('t [s]', fontsize=15)
	plt.ylabel('roll [°]', fontsize=15)


if __name__ == '__main__':
	parser = argparse.ArgumentParser()
	parser.add_argument("--real_fn", type=str)
	parser.add_argument("--sim_fn", type=str)
	args = parser.parse_args()

	# read data
	real_poses = np.loadtxt(args.real_fn)
	sim_poses = np.loadtxt(args.sim_fn)
	# remove indices
	if np.shape(real_poses)[1] == 9:
		real_poses = real_poses[:,1:]
	if np.shape(sim_poses)[1] == 9:
		sim_poses = sim_poses[:,1:]

	# Manipulate real traj
	print('[WARNING] Manipulating real traj. Make sure that this part of the code is up-to-date.')
	# time align
	real_poses[:,0] -= real_poses[0,0]
	sim_poses[:,0] -= sim_poses[0,0]
	# time shift
	t_shift = 36.25
	real_poses[:,0] -= t_shift
	# remove negative times
	real_poses = np.asarray([p for p in real_poses if p[0] >= 0.0])

	# compute rotations
	real_ori_zyx = getRotationEulerZYX(real_poses)
	sim_ori_zyx = getRotationEulerZYX(sim_poses)

	plt.figure('XY View')
	plotPosition2D(real_poses[:,1:3], sim_poses[:,1:3], 'x [m]', 'y [m]', "View X-Y")
	plt.figure('XZ View')
	plotPosition2D(real_poses[:,[1,3]], sim_poses[:,[1,3]], 'x [m]', 'z [m]', "View X-Z")
	plt.figure('YZ View')
	plotPosition2D(real_poses[:,2:4], sim_poses[:,2:4], 'y [m]', 'z [m]', "View Y-Z")
	plt.figure('XYZ Time')
	plotPosition1D(real_poses[:,0], real_poses[:,1:4], sim_poses[:,0], sim_poses[:,1:4], "Position")
	plt.figure('YPR Time')
	plotOrientation(real_poses[:,0], real_ori_zyx, sim_poses[:,0], sim_ori_zyx, "Orientation")

	plt.show()

