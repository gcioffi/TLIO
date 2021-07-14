''' 
Helper for plotting functions
'''

import numpy as np
import matplotlib.pyplot as plt


def plotArray(x, y, title, labels):
    plt.plot(x, y)
    plt.grid()
    plt.xlabel(labels[0])
    plt.ylabel(labels[1])
    plt.title(title)


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


def plotGtPosXY(pos):
    plt.plot(pos[:, 0], pos[:, 1])
    plt.grid()
    plt.xlabel('x')
    plt.ylabel('y')


def plotFilterVsGtPosXY(pos_filter, pos_gt):
    plt.plot(pos_filter[:, 0], pos_filter[:, 1], label='filter')
    plt.plot(pos_gt[:, 0], pos_gt[:, 1], label='gt')
    plt.grid()
    plt.legend()
    plt.xlabel('x')
    plt.ylabel('y')


def plotImuBiases(ts, biases):
    plt.plot(ts, biases[:, 0], label='x')
    plt.plot(ts, biases[:, 1], label='y')
    plt.plot(ts, biases[:, 2], label='z')
    plt.grid()
    plt.legend()
    plt.xlabel('t')
    plt.ylabel('bias')


# [in] ts = timestamps
# [in] errs = np.array[[err_x err_y, err_z], [...], ...]
def plotRotErrEulerXYZ(ts, errs):
    plt.plot(ts, errs[:, 0], label='x')
    plt.plot(ts, errs[:, 1], label='y')
    plt.plot(ts, errs[:, 2], label='z')
    plt.grid()
    plt.legend()
    plt.xlabel('t')
    plt.ylabel('errors')
    plt.title('Error euler angles')

