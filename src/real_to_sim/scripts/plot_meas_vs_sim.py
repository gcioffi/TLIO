import numpy as np
import matplotlib.pyplot as plt
import os 
import yaml


config_fn = os.path.abspath(os.getcwd()) + '/../../params/dataloader_params.yaml' 
with open(str(config_fn), 'r') as file:
	    conf = yaml.load(file, Loader=yaml.FullLoader)
folder_directory = conf["bagfile"]

imu_measurements = np.loadtxt(folder_directory + "/seq1/imu_measurements.txt")
sim_imu_measurements = np.loadtxt(folder_directory + "/seq1/2021-02-03-13-43-38_sim_imu_meas.txt")

# Extract Time
time = imu_measurements[:, 0]
time = np.reshape(time, (time.shape[0],1))

sim_time = sim_imu_measurements[:, 0]
sim_time = np.reshape(sim_time, (sim_time.shape[0],1))

# Extract IMU
w = imu_measurements[:, 7:10]
w = np.reshape(w, (w.shape[0], 3))

a = imu_measurements[:, 1:4]
a = np.reshape(a, (a.shape[0], 3))

w_sim = sim_imu_measurements[:, 1:4]
w_sim = np.reshape(w_sim, (w_sim.shape[0], 3))

a_sim = sim_imu_measurements[:, 4:7]
a_sim = np.reshape(a_sim, (a_sim.shape[0], 3))

# Plotting
fig, axs = plt.subplots(3, 2)
axs[0, 0].plot(time, a[:, 0], '-r', label = "Real ax")
axs[0, 0].plot(sim_time, a_sim[:, 0], '-b', label = "Simulated ax")
axs[0, 0].legend()
axs[1, 0].plot(time, a[:, 1], '-r', label = "Real ay")
axs[1, 0].plot(sim_time, a_sim[:, 1],  '-b', label = "Simulated ay")
axs[1, 0].legend()
axs[2, 0].plot(time, a[:, 2], '-r', label = "Real az")
axs[2, 0].plot(sim_time, a_sim[:, 2],  '-b', label = "Simulated az")
axs[2, 0].legend()
axs[0, 1].plot(time, w[:, 0], '-r', label = "Real wx")
axs[0, 1].plot(sim_time, w_sim[:, 0],  '-b', label = "Simulated wx")
axs[0, 1].legend()
axs[1, 1].plot(time, w[:, 1], '-r', label = "Real wy")
axs[1, 1].plot(sim_time, w_sim[:, 1],  '-b', label = "Simulated wy")
axs[1, 1].legend()
axs[2, 1].plot(time, w[:, 2], '-r', label = "Real wz")
axs[2, 1].plot(sim_time, w_sim[:, 2], '-b', label = "Simulated wz")
axs[2, 1].legend()
plt.show()
