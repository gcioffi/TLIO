import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
import scipy
from scipy.signal import savgol_filter
import os
import yaml


config_fn = os.path.abspath(os.getcwd()) + '/../params/dataloader_params.yaml' 
with open(str(config_fn), 'r') as file:
	    conf = yaml.load(file, Loader=yaml.FullLoader)
folder_directory = conf["bagfile"]
folder_directory_simulation = conf["bagfile_sim"]

a_real = np.loadtxt(folder_directory + "/seq1/imu_measurements.txt")
ts_real = a_real[:, 0]
ts_real = ts_real - ts_real[0]
a_real_raw = a_real[:, 1:4]
w_real_raw = a_real[:, 7:10]

a_gt = np.loadtxt(folder_directory_simulation + "/seq1/imu_measurements.txt")
ts_gt = a_gt[:, 0]
ts_gt = ts_gt - ts_gt[0]
a_gt_sim = a_gt[:, 1:4]
w_gt_sim = a_gt[:, 7:10]

#Biases in IMU Frame from Dynamic-Calibration
'''
acc_bias_real = np.array([0.002, 0.124, 0.103])
gyr_bias_real = np.array([0.002, -0.006, -0.0])

acc_bias_sim = np.array([0.089, -0.033, 0.058])
gyr_bias_sim = np.array([0.001, 0.006, 0.008 ])
'''

fig, axs = plt.subplots(3)
fig.suptitle('Real vs. Simulated GT Linear Acceleration')
axs[0].plot(ts_real, a_real_raw[:, 0], "r-", alpha=0.6, label = "X - Real")
axs[0].plot(ts_gt - 2.4, a_gt_sim[:, 0], "b-", alpha=0.55, label = "X - Simulated")
axs[0].legend()
axs[1].plot(ts_real, a_real_raw[:, 1], "r-", alpha=0.6, label = "Y - Real")
axs[1].plot(ts_gt - 2.4, a_gt_sim[:, 1], "b-", alpha=0.55, label = "Y - Simulated")
axs[1].legend()
axs[2].plot(ts_real, a_real_raw[:, 2], "r-", alpha=0.6, label = "Z - Real")
axs[2].plot(ts_gt - 2.4, a_gt_sim[:, 2], "b-", alpha=0.55, label = "Z - Simulated")
axs[2].legend()
plt.xlabel('t (s)')
plt.show()

# Simulated Gyro shifted by 1 sec for aligning
fig, axs = plt.subplots(3)
fig.suptitle('Real vs. Simulated GT Angular Velocity')
axs[0].plot(ts_real, w_real_raw[:, 0], "r-", alpha=0.6, label = "X - Real")
axs[0].plot(ts_gt - 2.4, w_gt_sim[:, 0], "b-", alpha=0.75, label = "X - Simulated")
axs[0].legend()
axs[1].plot(ts_real, w_real_raw[:, 1], "r-", alpha=0.6, label = "Y - Real")
axs[1].plot(ts_gt - 2.4, w_gt_sim[:, 1], "b-", alpha=0.75, label = "Y - Simulated")
axs[1].legend()
axs[2].plot(ts_real, w_real_raw[:, 2], "r-", alpha=0.6, label = "Z - Real")
axs[2].plot(ts_gt - 2.4, w_gt_sim[:, 2], "b-", alpha=0.75, label = "Z - Simulated")
axs[2].legend()
plt.xlabel('t (s)')
plt.show()
