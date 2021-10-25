'''
This script plots real and simulated imu measurements .

Notation:
- T: 6DoF transformation matrix. Pose class
- R: rotation matrix. (3,3) numpy vector  
- p: position vector. (3,) numpy vector 
- t: time. Scalar

Inputs:
- real_imu_fn.txt : Real imu measurements. Format [time_sec acc_x acc_y acc_z gyr_x gyr_y gyr_z]
- sim_imu_fn.txt : Simulated imu measurements. Format [time_sec acc_x acc_y acc_z gyr_x gyr_y gyr_z]

Outputs:
- None

'''


import argparse
import os

import IPython
import matplotlib.pyplot as plt
import numpy as np
from pyquaternion import Quaternion


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
	plt.plot(sim_imu_t, sim_imu_acc[:, 0], label='sim', alpha=0.75)
	plt.grid()
	plt.legend(fontsize=15)
	plt.xlabel('t [s]', fontsize=15)
	plt.ylabel('$a_{x}$ $[m/s^{2}]$', fontsize=15)
	plt.title('Linear accelerations',fontsize=20)

	plt.subplot(323)
	plt.plot(real_imu_t, real_imu_acc[:, 1], label='real')
	plt.plot(sim_imu_t, sim_imu_acc[:, 1], label='sim', alpha=0.75)
	plt.grid()
	plt.legend(fontsize=15)
	plt.xlabel('t [s]', fontsize=15)
	plt.ylabel('$a_{y}$ $[m/s^{2}]$', fontsize=15)

	plt.subplot(325)
	plt.plot(real_imu_t, real_imu_acc[:, 2], label='real')
	plt.plot(sim_imu_t, sim_imu_acc[:, 2], label='sim', alpha=0.75)
	plt.grid()
	plt.legend(fontsize=15)
	plt.xlabel('t [s]', fontsize=15)
	plt.ylabel('$a_{z}$ $[m/s^{2}]$', fontsize=15)

	# Plot gyro.
	plt.subplot(322)
	plt.plot(real_imu_t, real_imu_gyro[:, 0], label='real')
	plt.plot(sim_imu_t, sim_imu_gyro[:, 0], label='sim', alpha=0.75)
	plt.grid()
	plt.legend(fontsize=15)
	plt.xlabel('t [s]', fontsize=15)
	plt.ylabel('$ω_{x}$ $[rad/s]$', fontsize=15)
	plt.title('Angular velocities', fontsize=20)

	plt.subplot(324)
	plt.plot(real_imu_t, real_imu_gyro[:, 1], label='real')
	plt.plot(sim_imu_t, sim_imu_gyro[:, 1], label='sim', alpha=0.75)
	plt.grid()
	plt.legend(fontsize=15)
	plt.xlabel('t [s]', fontsize=15)
	plt.ylabel('$ω_{y}$ $[rad/s]$', fontsize=15)

	plt.subplot(326)
	plt.plot(real_imu_t, real_imu_gyro[:, 2], label='real')
	plt.plot(sim_imu_t, sim_imu_gyro[:, 2], label='sim', alpha=0.75)
	plt.grid()
	plt.legend(fontsize=15)
	plt.xlabel('t [s]', fontsize=15)
	plt.ylabel('$ω_{z}$ $[rad/s]$', fontsize=15)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("--real_imu_fn", type=str)
    parser.add_argument("--sim_imu_fn", type=str)
    args = parser.parse_args()
    
    # read measurements
    real_imu_fn = args.real_imu_fn
    sim_imu_fn = args.sim_imu_fn

    real_imu = np.loadtxt(real_imu_fn)
    sim_imu = np.loadtxt(sim_imu_fn)
    
    # Visualize
    title = 'Compare sim-real IMU measurements'
    plt.figure(title)
    #plotImu([real_imu[:, 0], real_imu[:,4:7], real_imu[:,1:4]], [sim_imu[:,0], sim_imu[:,4:7], sim_imu[:,1:4]])
    plotImu([real_imu[:, 0] - real_imu[0, 0], real_imu[:,4:7], real_imu[:,1:4]], [sim_imu[:,0] - sim_imu[0,0], sim_imu[:,4:7], sim_imu[:,1:4]])

    plt.show()

