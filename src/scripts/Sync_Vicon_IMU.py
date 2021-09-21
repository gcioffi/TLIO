import numpy as np
import matplotlib.pyplot as plt
import pandas as pd


imu = np.loadtxt("imu_measurements.txt")
ts_real = imu[:, 0]
print("Shape", ts_real.shape[0])
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


df.to_csv('LOG00001.csv', index=False)



