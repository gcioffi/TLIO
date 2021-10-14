'''
This script plots the fft of one axis measurement of real and imu measurements .

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
from scipy.fft import fft, fftshift
from scipy import signal


def plotFFT(x, freq, title):
	# fft
    y = fftshift(np.real(fft(x)))
    n = np.shape(x)[0]
    f = np.arange(-n/2, n/2) * float(freq/n)
    power = np.power(np.abs(y), 2) / n
    
    # plot
    f_p = np.array([v for v in f if v >= 0])
    power_p = np.array([power[i] for i,v in enumerate(f) if v >= 0])
    plt.loglog(f_p, power_p)
    plt.title(title)
    plt.ylim([1e-15, 1e4])
    plt.xlabel('Frequency [Hz]')
    plt.ylabel('|FFT (x)|^2')
    plt.grid()


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("--real_imu_fn", type=str)
    parser.add_argument("--sim_imu_fn", type=str)
    parser.add_argument("--freq", type=float, help='freq of the signals in Hz')
    args = parser.parse_args()
    
    # read measurements
    freq = args.freq
    real_imu_fn = args.real_imu_fn
    sim_imu_fn = args.sim_imu_fn
    real_imu = np.loadtxt(real_imu_fn)
    sim_imu = np.loadtxt(sim_imu_fn)

    # plot fft
    plt.figure('Gyro FFT Power Spectrum')
    plt.subplot(321)
    plotFFT(real_imu[:,1], 400, 'Real wx')
    plt.subplot(322)
    plotFFT(sim_imu[:,1], 1000, 'Sim wx')
    plt.subplot(323)
    plotFFT(real_imu[:,2], 400, 'Real wy')
    plt.subplot(324)
    plotFFT(sim_imu[:,2], 1000, 'Sim wy')
    plt.subplot(325)
    plotFFT(real_imu[:,3], 400, 'Real wz')
    plt.subplot(326)
    plotFFT(sim_imu[:,3], 1000, 'Sim wz')

    plt.figure('Accel. FFT Power Spectrum')
    plt.subplot(321)
    plotFFT(real_imu[:,4], 400, 'Real ax')
    plt.subplot(322)
    plotFFT(sim_imu[:,4], 1000, 'Sim ax')
    plt.subplot(323)
    plotFFT(real_imu[:,5], 400, 'Real ay')
    plt.subplot(324)
    plotFFT(sim_imu[:,5], 1000, 'Sim ay')
    plt.subplot(325)
    plotFFT(real_imu[:,6], 400, 'Real az')
    plt.subplot(326)
    plotFFT(sim_imu[:,6], 1000, 'Sim az')

    plt.show()

