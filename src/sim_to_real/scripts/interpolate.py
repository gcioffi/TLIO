import numpy as np
import os
import yaml

config_fn = os.path.abspath(os.getcwd()) + '/../../params/dataloader_params.yaml' 
with open(str(config_fn), 'r') as file:
	    conf = yaml.load(file, Loader=yaml.FullLoader)
folder_directory = conf["bagfile"]

evolving_state = np.loadtxt(folder_directory + "/seq1/evolving_state.txt")
noisy_imu_measurements = np.loadtxt(folder_directory + "/seq1/imu_measurements.txt")
my_timestamps_p = np.loadtxt(folder_directory + "/seq1/my_timestamps_p.txt")

# Interpolate IMU at 1000 Hz
noisy_ts_imu = []
noisy_imu_measurements_interp = []
dt_interp = 0.001 * 1e6

for imu_idx in range(noisy_imu_measurements.shape[0]):
    if imu_idx == 0:
        noisy_ts_imu.append(noisy_imu_measurements[imu_idx, 0])
        noisy_imu_measurements_interp.append(np.array(noisy_imu_measurements[imu_idx, :]))

    else:
        prev_noisy_ts_imu = noisy_ts_imu[-1]
        prev_noisy_ts_fixed = prev_noisy_ts_imu
        curr_noisy_ts_imu = noisy_imu_measurements[imu_idx, 0]

        curr_noisy_imu_measurements = noisy_imu_measurements[imu_idx, :]
        prev_noisy_imu_measurements = np.array(noisy_imu_measurements_interp[-1])

        while (prev_noisy_ts_imu + dt_interp) < (curr_noisy_ts_imu - 0.0001*1e6):
            dt = dt_interp
            t = prev_noisy_ts_imu + dt
        
            interp_noisy_imu_measurements = prev_noisy_imu_measurements + \
                ((curr_noisy_imu_measurements - prev_noisy_imu_measurements) / (curr_noisy_ts_imu - prev_noisy_ts_fixed)) \
                    * (t - prev_noisy_ts_fixed)

            noisy_ts_imu.append(t)
            noisy_imu_measurements_interp.append(np.array(interp_noisy_imu_measurements))
        

            prev_noisy_ts_imu += dt_interp
            
        noisy_ts_imu.append(curr_noisy_ts_imu)
        noisy_imu_measurements_interp.append(np.array(curr_noisy_imu_measurements))

# Interpolate evolving at 1000 Hz
noisy_evolving_state_interp = []
noisy_ts_odom = []
dt_interp = 0.001 * 1e6

for odom_idx in range(evolving_state.shape[0]):
    if odom_idx == 0:
        noisy_ts_odom.append(evolving_state[odom_idx, 0])
        noisy_evolving_state_interp.append(np.array(evolving_state[odom_idx, :]))

    else:
        prev_noisy_ts_odom = noisy_ts_odom[-1]
        prev_noisy_ts_fixed = prev_noisy_ts_odom
        curr_noisy_ts_odom = evolving_state[odom_idx, 0]

        curr_evolving_state = evolving_state[odom_idx, :]
        prev_evolving_state = np.array(noisy_evolving_state_interp[-1])

        while (prev_noisy_ts_odom + dt_interp) < (curr_noisy_ts_odom - 0.0001 * 1e6):
            dt = dt_interp
            t = prev_noisy_ts_odom + dt
        
            interp_evolving_state = prev_evolving_state + \
                ((curr_evolving_state - prev_evolving_state) / (curr_noisy_ts_odom - prev_noisy_ts_fixed)) \
                    * (t - prev_noisy_ts_fixed)

            noisy_ts_odom.append(t)
            noisy_evolving_state_interp.append(np.array(interp_evolving_state))
        

            prev_noisy_ts_odom += dt_interp
            
        noisy_ts_odom.append(curr_noisy_ts_odom)
        noisy_evolving_state_interp.append(np.array(curr_evolving_state))

noisy_imu_measurements_interp = np.array(noisy_imu_measurements_interp)
noisy_evolving_state_interp = np.array(noisy_evolving_state_interp)
noisy_ts_odom = np.array(noisy_ts_odom)

np.savetxt(folder_directory + "/seq1/noisy_evolving_state.txt", noisy_evolving_state_interp)
np.savetxt(folder_directory + "/seq1/noisy_imu_measurements.txt", noisy_imu_measurements_interp)
np.savetxt(folder_directory + "/seq1/noisy_my_timestamps_p.txt", noisy_ts_odom)
