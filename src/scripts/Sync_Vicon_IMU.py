import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
import yaml
import os

bag_name = '10_48_03.bag'

# Get config directory
config_fn = os.path.abspath(os.getcwd()) + '/../params/dataloader_params.yaml' 
# e.g.: /home/rpg/Desktop/TLIO/src/scripts/../params/dataloader_params.yaml

with open(str(config_fn), 'r') as file:
	    conf = yaml.load(file, Loader=yaml.FullLoader)

# Get folder directory
folder_directory = conf["bagfile"]
# e.g.: /home/rpg/Desktop/RosbagReal_10_48_03

# Get bag directory
bag_directory = os.path.join(conf["bagfile"], bag_name) 
# e.g.: /home/rpg/Desktop/RosbagReal_10_48_03/10_48_03.bag

# Load imu_measurements
imu = np.loadtxt(os.path.join(folder_directory, 'seq1', 'imu_measurements.txt'))
ts_real = imu[:, 0] * 1e-6 # seconds 
ax_real = imu[:, 1]
ay_real = imu[:, 2]
az_real = imu[:, 3]
a = np.zeros(shape=(ts_real.shape[0],3))
a[:,0] = ax_real
a[:,1] = ay_real
a[:,2] = az_real
norm = np.linalg.norm(a, axis=1)
ts = ts_real

df = pd.DataFrame(
    {
        "ts": ts,
        "t": ts - ts[0],
        "an": norm,

    }
)

# Save file needed for synchronization
df.to_csv(os.getcwd() + '/../Vicon/data/LOG00001.csv', index=False)




