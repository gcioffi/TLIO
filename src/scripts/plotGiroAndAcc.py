import argparse
import matplotlib.pyplot as plt
import numpy as np
import os

from tlio.plots import plotImuCalibVsRaw, plotGtPosXY


def plotData(data_dir):
    # Load data
    imu_fn = os.path.join(data_dir, 'imu_measurements.txt')
    gt_fn = os.path.join(data_dir, 'evolving_state.txt')

    imu_data = np.loadtxt(imu_fn)
    gt_data = np.loadtxt(gt_fn)

    t_imu = imu_data[:, 0]
    acc_raw = imu_data[:, 1:4]
    acc_calib = imu_data[:, 4:7]
    gyro_raw = imu_data[:, 7:10]
    gyro_calib = imu_data[:, 10:13]

    t_gt = gt_data[:, 0]
    p_gt = gt_data[:, 5:8]

    fig = plt.figure(0)
    plotImuCalibVsRaw(t_imu, acc_calib, acc_raw, 'acc')
    fig = plt.figure(1)
    plotImuCalibVsRaw(t_imu, gyro_calib, gyro_raw, 'gyro')

    #fig = plt.figure(2)
    #plotGtPosXY(t_gt, p_gt)

    plt.show()


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("--data_dir", type=str, default='/home/rpg/Desktop/TLIO/data/Dataset/seq1')
    args = parser.parse_args()
    data_dir = args.data_dir
    plotData(data_dir)

