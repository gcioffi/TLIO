import numpy as np
import matplotlib.pyplot as plt
import os
import yaml

config_fn = os.path.abspath(os.getcwd()) + '/../../params/dataloader_params.yaml' 
with open(str(config_fn), 'r') as file:
	    conf = yaml.load(file, Loader=yaml.FullLoader)
folder_directory = conf["bagfile"]

# Check break condition - you may need to add the last elements in imu or evolving before breaking
vio_states = np.loadtxt(folder_directory + "/seq1/noisy_evolving_state_interp.txt")
ts = vio_states[:, 0] 
pos = vio_states[:, 5:8]

imu = np.loadtxt(folder_directory + "/seq1/noisy_imu_measurements_interp.txt")
imu_ts = imu[:, 0] 

j = 0
i = 0
counter = 0
vio_states_new = []
imu_new = []
interp = 0
break_ = False
idx_evolving_corresp = []
idx_imu_corresp = []

while i <= pos.shape[0] - 1 and j <= imu_ts.shape[0] - 1:
    
    break_ = False

    if(ts[i] > imu_ts[j]):
        imu_new.append(imu[j])
        j = j + 1
        if j > imu_ts.shape[0] - 1:
            while i <= pos.shape[0] - 1:
                vio_states_new.append(vio_states[i])
                i += 1
        continue

    if ts[i] == imu_ts[j]:
        vio_states_new.append(vio_states[i])
        idx_evolving_corresp.append(len(vio_states_new) - 1)
        imu[j, 13] = 1
        imu_new.append(imu[j])
        idx_imu_corresp.append(len(imu_new) - 1)

        j += 1
        i += 1
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
        if break_ == True: continue

  
    if ts[i] < imu_ts[j]:
        while(ts[i] < imu_ts[j]):
            vio_states_new.append(vio_states[i])
            i +=1
            if i > pos.shape[0] - 1:
                while j <= imu_ts.shape[0] - 1:
                    imu_new.append(imu[j])
                    j += 1
                break_ = True
            if break_ == True: break
        if break_ == True: break
     
        if ts[i] == imu_ts[j]:
            vio_states_new.append(vio_states[i])
            idx_evolving_corresp.append(len(vio_states_new) - 1)
            imu[j, 13] = 1
            imu_new.append(imu[j])
            idx_imu_corresp.append(len(imu_new) - 1)
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
            if break_ == True: continue

        if ts[i] > imu_ts[j]:
            tmp = vio_states[i-1] + ((vio_states[i]- vio_states[i-1])/(ts[i] - ts[i-1])) * (imu_ts[j] - ts[i-1])
            vio_states_new.append(tmp)
            idx_evolving_corresp.append(len(vio_states_new) - 1)
            interp += 1
            vio_states_new.append(vio_states[i])
            imu[j, 13] = 1
            imu_new.append(imu[j])
            idx_imu_corresp.append(len(imu_new) - 1)
            
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
            if break_ == True: continue



vio_states_new = np.asarray(vio_states_new)
imu_new = np.asarray(imu_new)
idx_imu_corresp = np.array(idx_imu_corresp)
idx_evolving_corresp = np.array(idx_evolving_corresp)

''''
# Debug
print("In for loop!")
print("Shape", idx_imu_corresp.shape[0])
for k in range(idx_imu_corresp.shape[0]):
    print(imu_new[idx_imu_corresp[k], 0], vio_states_new[idx_evolving_corresp[k], 0])
'''

np.savetxt(folder_directory + "/seq1/noisy_evolving_state_interp.txt", vio_states_new)
np.savetxt(folder_directory + "/seq1/noisy_imu_measurements_interp.txt", imu_new)
np.savetxt(folder_directory + "/seq1/idx_imu_corresp.txt", idx_imu_corresp)
np.savetxt(folder_directory + "/seq1/idx_evolving_corresp.txt", idx_evolving_corresp)

print("Shape VIO -> new: ", vio_states_new.shape[0], " old: ", vio_states.shape[0])
print("Shape IMU -> new: ", imu_new.shape[0], "old: ", imu.shape[0]) 
print("Number of interpolations: ", interp)
