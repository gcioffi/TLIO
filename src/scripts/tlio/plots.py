''' 
Helper for plotting functions
'''

import numpy as np
import matplotlib.pyplot as plt


def plotImu(times, meas, sens):
    plt.subplot(311)
    plt.plot(times, meas[:, 0], label='x')
    plt.grid()
    plt.legend()
    plt.xlabel('t')
    plt.ylabel('x')

    if sens == 'acc':
        plt.title('Acc. measurements')
    else:
        plt.title('Gyro. measurements')
    
    plt.subplot(312)
    plt.plot(times, meas[:, 1], label='y')
    plt.grid()
    plt.legend()
    plt.xlabel('t')
    plt.ylabel('y')

    plt.subplot(313)
    plt.plot(times, meas[:, 2], label='z')
    plt.grid()
    plt.legend()
    plt.xlabel('t')
    plt.ylabel('z')


def plotImuCalibVsRaw(times, meas_calib, meas_raw, sens):
    plt.subplot(311)
    plt.plot(times, meas_calib[:, 0], label='calib')
    plt.plot(times, meas_raw[:, 0], label='raw')
    plt.grid()
    plt.legend()
    plt.xlabel('t')
    plt.ylabel('x')

    if sens == 'acc':
        plt.title('Acc. measurements')
    else:
        plt.title('Gyro. measurements')
    
    plt.subplot(312)
    plt.plot(times, meas_calib[:, 1], label='calib')
    plt.plot(times, meas_raw[:, 1], label='raw')
    plt.grid()
    plt.legend()
    plt.xlabel('t')
    plt.ylabel('y')

    plt.subplot(313)
    plt.plot(times, meas_calib[:, 2], label='calib')
    plt.plot(times, meas_raw[:, 2], label='raw')
    plt.grid()
    plt.legend()
    plt.xlabel('t')
    plt.ylabel('z')


def plotGtPosXY(times, pos):
    plt.plot(pos[:, 0], pos[:, 1])
    plt.grid()
    plt.xlabel('x')
    plt.ylabel('y')

