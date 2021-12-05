''' 
Helper for plotting functions
'''

import numpy as np
import matplotlib.pyplot as plt


def plotArray(x, y, title, labels):
    plt.plot(x, y)
    plt.xticks(fontsize=30)
    plt.yticks(fontsize=30)
    plt.grid()
    plt.xlabel(labels[0], fontsize = 30, labelpad=15)
    plt.ylabel(labels[1], fontsize = 30, labelpad=15)
    plt.title(title, fontsize = 35, pad = 23)

def plotImu(times, meas, sens):
    plt.subplot(311)
    plt.plot(times, meas[:, 0], label='x')
    plt.grid()
    plt.legend(fontsize = 15)
    plt.xlabel('t [s]', fontsize = 15)
    plt.ylabel('x', fontsize = 15)

    if sens == 'acc':
        plt.title('Acc. measurements', fontsize = 20)
    else:
        plt.title('Gyro. measurements', fontsize = 20)
    
    plt.subplot(312)
    plt.plot(times, meas[:, 1], label='y')
    plt.grid()
    plt.legend(fontsize = 15)
    plt.xlabel('t [s]', fontsize = 15)
    plt.ylabel('y', fontsize = 15)

    plt.subplot(313)
    plt.plot(times, meas[:, 2], label='z')
    plt.grid()
    plt.legend(fontsize = 15)
    plt.xlabel('t [s]', fontsize = 15)
    plt.ylabel('z', fontsize = 15)


def plotImuCalibVsRaw(times, meas_calib, meas_raw, sens):
    plt.subplot(311)
    plt.plot(times, meas_calib[:, 0], label='calib')
    plt.plot(times, meas_raw[:, 0], label='raw')
    plt.grid()
    plt.legend(fontsize = 15)
    plt.xlabel('t', fontsize = 15)
    plt.ylabel('x', fontsize = 15)

    if sens == 'acc':
        plt.title('Acc. measurements', fontsize = 20)
    else:
        plt.title('Gyro. measurements', fontsize = 20)
    
    plt.subplot(312)
    plt.plot(times, meas_calib[:, 1], label='calib')
    plt.plot(times, meas_raw[:, 1], label='raw')
    plt.grid()
    plt.legend(fontsize = 15)
    plt.xlabel('t',fontsize = 15)
    plt.ylabel('y', fontsize = 15)

    plt.subplot(313)
    plt.plot(times, meas_calib[:, 2], label='calib')
    plt.plot(times, meas_raw[:, 2], label='raw')
    plt.grid()
    plt.legend(fontsize = 15)
    plt.xlabel('t', fontsize = 15)
    plt.ylabel('z',fontsize = 15)


def plotGtPosXY(pos):
    plt.plot(pos[:, 0], pos[:, 1])
    plt.grid()
    plt.xlabel('x', fontsize = 15)
    plt.ylabel('y', fontsize = 15)


def plotFilterVsGtPosXY(pos_filter, pos_gt):
    plt.plot(pos_filter[:, 0], pos_filter[:, 1], label='EKF Estimate')
    plt.plot(pos_gt[:, 0], pos_gt[:, 1], label='Ground-Truth')
    plt.grid()
    plt.legend(fontsize = 30)
    plt.xticks(fontsize=30)
    plt.yticks(fontsize=30)
    plt.title("Filter vs. Ground-Truth Position",  fontsize = 35, pad = 23)
    plt.xlabel('x [m]', fontsize = 30, labelpad=15)
    plt.ylabel('y [m]', fontsize = 30)


def plotImuBiases(ts, biases, name):
    plt.plot(ts, biases[:, 0], label='x')
    plt.plot(ts, biases[:, 1], label='y')
    plt.plot(ts, biases[:, 2], label='z')
    plt.grid()
    plt.legend(fontsize = 30)
    plt.xticks(fontsize=30)
    plt.yticks(fontsize=30)
    plt.xlabel('t [s]', fontsize = 30, labelpad=15)
    if name == "ba":
        plt.ylabel('$b_{a}$ [m/s$^{2}$]', fontsize = 30, labelpad=15)
        plt.title("Accelerometer Bias",  fontsize = 35, pad = 23)
        plt.ticklabel_format(axis="y", style="plain", scilimits=(0,0))
    if name == "bg":
        plt.ylabel('$b_{g}$ [rad/s]', fontsize = 30, labelpad=15)
        plt.title("Gyroscope Bias",  fontsize = 35, pad = 23)
        plt.ticklabel_format(axis="y", style="plain", scilimits=(0,0))


# [in] ts = timestamps
# [in] errs = np.array[[err_x err_y, err_z], [...], ...]
def plotRotErrEulerXYZ(ts, errs):
    plt.plot(ts, errs[:, 0], label='x')
    plt.plot(ts, errs[:, 1], label='y')
    plt.plot(ts, errs[:, 2], label='z')
    plt.grid()
    plt.legend(fontsize = 30)
    plt.xticks(fontsize=30)
    plt.yticks(fontsize=30)
    plt.xlabel('t [s]', fontsize = 30, labelpad=15)
    plt.ylabel('Errors [°]', fontsize = 30, labelpad=15)
    plt.title('Error Euler Angles', fontsize = 35, pad = 23)

