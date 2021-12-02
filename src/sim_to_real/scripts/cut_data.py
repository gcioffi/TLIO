import numpy as np
import matplotlib.pyplot as plt
import os
import yaml

# Cut data

config_fn = os.path.abspath(os.getcwd()) + '/../../params/dataloader_params.yaml' 
with open(str(config_fn), 'r') as file:
	    conf = yaml.load(file, Loader=yaml.FullLoader)
folder_directory = conf["bagfile"]

def find_nearest(array, value):
    array = np.asarray(array)
    idx = (np.abs(array - value)).argmin()
    return idx

imu_measurements = np.loadtxt(folder_directory + "/seq1/imu_measurements.txt")
my_timestamps_p = np.loadtxt(folder_directory + "/seq1/my_timestamps_p.txt")
evolving_state = np.loadtxt(folder_directory + "/seq1/evolving_state.txt")

t_imu = imu_measurements[:, 0] * 1e-6 # seconds
# Insert here time beginning - end
idx_imu_begin = find_nearest(t_imu, t_imu[0] + 34.2) # cut of rosbag to have 2 secs of hover 
idx_imu_end = find_nearest(t_imu, t_imu[0] + 67.8)
t_odom = my_timestamps_p * 1e-6 # seconds
idx_odom_begin = find_nearest(t_odom, t_imu[idx_imu_begin])
idx_odom_end = find_nearest(t_odom, t_imu[idx_imu_end])

imu_measurements_ = imu_measurements[idx_imu_begin:idx_imu_end+1, :]
evolving_state_ = evolving_state[idx_odom_begin:idx_odom_end+1, :]
my_timestamps_p_ = evolving_state_[:, 0]

# Save in microsecs
np.savetxt(folder_directory + "/seq1/imu_measurements.txt", imu_measurements_)
np.savetxt(folder_directory + "/seq1/evolving_state.txt", evolving_state_)
np.savetxt(folder_directory + "/seq1/my_timestamps_p.txt", my_timestamps_p_)

'''
# Plotting

fig, axs = plt.subplots(3, 2)
axs[0, 0].plot(t_imu * 1e6, imu_measurements[:, 1])
axs[0, 0].plot(imu_measurements_[:, 0], imu_measurements_[:, 1])

axs[1, 0].plot(t_imu * 1e6,, imu_measurements[:, 2])
axs[1, 0].plot(imu_measurements_[:, 0], imu_measurements_[:, 2])

axs[2, 0].plot(t_imu * 1e6,, imu_measurements[:, 3])
axs[2, 0].plot(imu_measurements_[:, 0], imu_measurements_[:, 3])

axs[0, 1].plot(t_imu * 1e6,, imu_measurements[:, 7])
axs[0, 1].plot(imu_measurements_[:, 0], imu_measurements_[:, 7])

axs[1, 1].plot(t_imu * 1e6,, imu_measurements[:, 8])
axs[1, 1].plot(imu_measurements_[:, 0], imu_measurements_[:, 8])

axs[2, 1].plot(t_imu * 1e6,, imu_measurements[:, 9])
axs[2, 1].plot(imu_measurements_[:, 0], imu_measurements_[:, 9])

plt.show()

'''