import numpy as np
import matplotlib.pyplot as plt
import os
import yaml

config_fn = os.path.abspath(os.getcwd()) + '/../../params/dataloader_params.yaml' 
with open(str(config_fn), 'r') as file:
	    conf = yaml.load(file, Loader=yaml.FullLoader)
folder_directory = conf["bagfile"]

flown_imu = np.loadtxt(folder_directory + "/seq1/imu_measurements.txt")
np.savetxt(folder_directory + "/seq1/imu_measurements_flown.txt", flown_imu)

sim_imu_measurements = np.loadtxt(folder_directory + "/seq1/2021-02-03-13-43-38_sim_imu_meas.txt")
t_imu = sim_imu_measurements[:, 0]
t_imu = np.reshape(t_imu, (t_imu.shape[0],1))
w = sim_imu_measurements[:, 1:4]
a = sim_imu_measurements[:, 4:7]

imu_meas = np.hstack((t_imu, a, a, w, w, np.zeros((t_imu.shape[0], t_imu.shape[1]))))
np.savetxt(folder_directory + "/seq1/imu_measurements.txt", imu_meas)


