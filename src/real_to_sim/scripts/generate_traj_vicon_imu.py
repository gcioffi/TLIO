import numpy as np
import matplotlib.pyplot as plt
import os
import yaml

def find_nearest(array, value):
    array = np.asarray(array)
    idx = (np.abs(array - value)).argmin()
    return idx

config_fn = os.path.abspath(os.getcwd()) + '/../../params/dataloader_params.yaml' 
with open(str(config_fn), 'r') as file:
	    conf = yaml.load(file, Loader=yaml.FullLoader)
folder_directory = conf["bagfile"]

evolving_state = np.loadtxt(folder_directory + "/seq1/evolving_state.txt")

# time, px, py, pz, qx, qy, qz, qw
time = evolving_state[:, 0]
time = np.reshape(time, (time.shape[0], 1))
p = evolving_state[:, 5:8]
p = np.reshape(p, (p.shape[0], 3))
tmp1 = evolving_state[:, 2:5]
tmp2 = evolving_state[:, 1]
tmp2 = np.reshape(tmp2, (tmp2.shape[0], 1))

traj_vicon_imu = np.hstack((time, p, tmp1, tmp2))
np.savetxt(folder_directory + "/seq1/2021-02-03-13-43-38_traj_vicon_imu.txt", traj_vicon_imu)



