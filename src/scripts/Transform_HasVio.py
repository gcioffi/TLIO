import numpy as np
import matplotlib.pyplot as plt
import os
import yaml

#Check break condition - you may need to add the last elements in imu or evolving before breaking


config_fn = os.path.abspath(os.getcwd()) + '/../params/dataloader_params.yaml' 
with open(str(config_fn), 'r') as file:
	    conf = yaml.load(file, Loader=yaml.FullLoader)
folder_directory = conf["bagfile"]


vio_states = np.loadtxt(folder_directory + "/seq1/evolving_state.txt")
ts = vio_states[:, 0]
pos = vio_states[:, 5:8]

imu = np.loadtxt(folder_directory + "/seq1/imu_measurements.txt")
imu_ts = imu[:, 0]

j = 0
i = 0
counter = 0
vio_states_new = []
imu_new = []
interp = 0
break_ = False

while i <= pos.shape[0] - 1 and j <= imu_ts.shape[0] - 1:

    if(ts[i] > imu_ts[j]):
        imu_new.append(imu[j])
        j = j + 1
        if j > imu_ts.shape[0] - 1:
            while i <= pos.shape[0] - 1:
                vio_states_new.append(vio_states[i])
                i += 1
            break

    if ts[i] == imu_ts[j]:
        vio_states_new.append(vio_states[i])
        i += 1
        imu[j, 13] = 1
        imu_new.append(imu[j])
        j += 1
        if i > pos.shape[0] - 1:
            while j <= imu_ts.shape[0] - 1:
                imu_new.append(imu[j])
                j += 1
            break_ = True
        if j > imu_ts.shape[0] -1:
            while i <= pos.shape[0] - 1:
                vio_states_new.append(vio_states[i])
                i += 1
            break_ = True
        if break_ == True: break

    if ts[i] < imu_ts[j]:
        while(ts[i] < imu_ts[j]):
            vio_states_new.append(vio_states[i])
            i +=1
            if i > pos.shape[0] - 1:
                while j <= imu_ts.shape[0] - 1:
                    imu_new.append(imu[j])
                    j += 1
                break

        if ts[i] == imu_ts[j]:
            vio_states_new.append(vio_states[i])
            imu[j, 13] = 1
            imu_new.append(imu[j])
            i += 1
            j += 1
            if i > pos.shape[0] - 1:
                while j <= imu_ts.shape[0] - 1:
                    imu_new.append(imu[j])
                    j += 1
                break_ = True
            if j > imu_ts.shape[0] - 1:
                while i <= pos.shape[0] - 1:
                    vio_states_new.append(vio_states[i])
                    i += 1
                break_ = True
            if break_ == True: break

        if ts[i] > imu_ts[j]:
            tmp = vio_states[i-1] + ((vio_states[i] - vio_states[i-1])/(ts[i] - ts[i-1])) * (imu_ts[j] - ts[i-1])
            vio_states_new.append(tmp)
            interp += 1
            vio_states_new.append(vio_states[i])
            imu[j, 13] = 1
            imu_new.append(imu[j])
            i += 1
            j += 1
            if i > pos.shape[0] - 1:
                while j <= imu_ts.shape[0] - 1:
                    imu_new.append(imu[j])
                    j += 1
                break_ = True
            if j > imu_ts.shape[0] - 1:
                while i <= pos.shape[0] - 1:
                    vio_states_new.append(vio_states[i])
                    i += 1
                break_ = True
            if break_ == True: break


vio_states_new = np.asarray(vio_states_new)
imu_new = np.asarray(imu_new)

np.savetxt(folder_directory + "/seq1/evolving_state.txt", vio_states_new)
np.savetxt(folder_directory + "/seq1/imu_measurements.txt", imu_new)

print(vio_states_new.shape[0], vio_states.shape[0])
print(imu_new.shape[0], imu.shape[0]) 
