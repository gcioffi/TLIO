import numpy as np
import os

#file = "/home/rpg/Desktop/train_original.txt"
file = "/home/rpg/Desktop/val.txt"
train_original = np.loadtxt(file, dtype=str)
train_output = []

#Take only half of the sequences
'''for i in range(train_original.shape[0]):
    if i%2 == 0:
        train_output.append(train_original[i])
'''

#Take a certain number (counter) of seqences with length < 60 secs
'''counter = -1

for i in range(train_original.shape[0]):

    if counter < 500:
        timestamps = []
        address_ts = "/home/rpg/Desktop/TLIO/data/Dataset_Multiple_Traj"
        address_ts = os.path.join(address_ts, train_original[i])
        timestamps = np.loadtxt(os.path.join(address_ts, "my_timestamps_p.txt"))
        rosbag_length = timestamps[-1] - timestamps[0]
        if rosbag_length <= 60:
            train_output.append(train_original[i])
            counter = counter + 1
            print("rosbag_length",  train_original[i], rosbag_length)
        else:
            continue
    else:
        break

train_output = np.asarray(train_output)
np.savetxt('/home/rpg/Desktop/train_output.txt', train_output, fmt='%s')
'''

#Take all sequences with length < 60 secs
for i in range(train_original.shape[0]):
    timestamps = []
    address_ts = "/home/rpg/Desktop/TLIO/data/Dataset_Multiple_Traj"
    address_ts = os.path.join(address_ts, train_original[i])
    timestamps = np.loadtxt(os.path.join(address_ts, "my_timestamps_p.txt"))
    rosbag_length = timestamps[-1] - timestamps[0]
    if rosbag_length <= 60:
        train_output.append(train_original[i])
        #print("rosbag_length",  train_original[i], rosbag_length)
    else:
        continue

train_output = np.asarray(train_output)
#np.savetxt('/home/rpg/Desktop/train_output.txt', train_output, fmt='%s')
np.savetxt('/home/rpg/Desktop/val_output.txt', train_output, fmt='%s')
