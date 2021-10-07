import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
import scipy
from scipy.signal import savgol_filter
import os
import yaml

# Replace evolving state of the real trajectory where the velocity is null and insert the data coming from Vicon Sync script

v_wb_real = np.loadtxt(os.getcwd() + "/../Vicon/data/ViconVelocity.txt") # velocity center of the markers in vicon world
config_fn = os.path.abspath(os.getcwd()) + '/../params/dataloader_params.yaml' 
with open(str(config_fn), 'r') as file:
	    conf = yaml.load(file, Loader=yaml.FullLoader)
folder_directory = conf["bagfile"]
folder_directory_simulation = conf["bagfile_sim"]

evolving_state = np.loadtxt(folder_directory + "/seq1/evolving_state.txt") # Original
evolving_state[:, 8:11] = v_wb_real
np.savetxt(folder_directory + "/seq1/evolving_state.txt", evolving_state) # Original with Vicon velocity smoothed

v_wb = evolving_state[:, 8:11] # related to imu ts at 400 Hz
p_wb = evolving_state[:, 5:8] # related to imu ts at 400 Hz
ts_real = evolving_state[:, 0] - evolving_state[0, 0]  # ts di evolving state real -> no problem to use ts_evolving or ts_imu: they are =

evolving_state_sim = np.loadtxt(folder_directory_simulation + "/seq1/evolving_state.txt")
v_wb_sim = evolving_state_sim[:, 8:11]
p_wb_sim = evolving_state_sim[:, 5:8]
ts_sim = evolving_state_sim[:, 0] - evolving_state_sim[0, 0]

# Plot for visualization Vicon vs. Simulated
v_wb_x = v_wb[:, 0]
v_wb_y = v_wb[:, 1]
v_wb_z = v_wb[:, 2]
p_wb_x = p_wb[:, 0]
p_wb_y = p_wb[:, 1]
p_wb_z = p_wb[:, 2]

# Add the offset: here - 5.1
fig, axs = plt.subplots(3)
fig.suptitle('Vicon vs. SIM Velocity')
axs[0].plot(ts_real, v_wb_x, "r-", alpha=0.6, label = "vx - VICON")
axs[0].plot(ts_sim - 0.5, v_wb_sim[:, 0], "b-", alpha=0.75, label = "vx - SIM")
axs[0].legend()
axs[0].grid()
axs[1].plot(ts_real, v_wb_y, "r-", alpha=0.6, label = "vy - VICON")
axs[1].plot(ts_sim - 0.5, v_wb_sim[:, 1], "b-", alpha=0.75, label = "vy - SIM")
axs[1].legend()
axs[1].grid()
axs[2].plot(ts_real, v_wb_z, "r-", alpha=0.6, label = "vz - VICON")
axs[2].plot(ts_sim - 0.5, v_wb_sim[:, 2], "b-", alpha=0.75, label = "vz - SIM")
axs[2].legend()
axs[2].grid()
plt.xlabel('t (s)')
plt.show()

# Add the offset: 

fig, axs = plt.subplots(3)
fig.suptitle('Vicon vs. sim Position')
axs[0].plot(ts_real, p_wb_x, "r-", alpha=0.6, label = "x - VICON")
axs[0].plot(ts_sim - 0.5, p_wb_sim[:, 0], "b-", alpha=0.75, label = "x - SIM")
axs[0].legend()
axs[0].grid()
axs[1].plot(ts_real, p_wb_y, "r-", alpha=0.6, label = "y - VICON")
axs[1].plot(ts_sim - 0.5, p_wb_sim[:, 1], "b-", alpha=0.75, label = "y - SIM")
axs[1].legend()
axs[1].grid()
axs[2].plot(ts_real, p_wb_z, "r-", alpha=0.6, label = "z - VICON")
axs[2].plot(ts_sim - 0.5, p_wb_sim[:, 2], "b-", alpha=0.75, label = "z - SIM")
axs[2].legend()
axs[2].grid()

plt.xlabel('t (s)')
plt.show()
