# Interpola misure che gia abbiamo: f = interp1d(x, y)
#dove x sono il numero di ts e y e' il corrispondente valore

# extract measurement every 0.001 secondo dal primo valore
from scipy.interpolate import interp1d
import numpy as np
import matplotlib.pyplot as plt
import os
import yaml


def interpolate_meas(data):
    counter = 0
    interpolated_line = []
    continue_ = True

    while continue_ == True:
        previous_line =  data[counter]

        #Measure at 0% interval
        interpolated_line.append(previous_line)
        if counter >= data.shape[0] - 1:
            continue_ = False
            break

        #Measure at 20% interval
        counter += 1
        next_line = data[counter]
        interpolated_line_value = (next_line - previous_line) * 0.8 + previous_line
        interpolated_line.append(interpolated_line_value)
        previous_line = next_line
        if counter >= data.shape[0] - 1:
            continue_ = False
            break

        #Measure at 40% interval
        counter += 1
        next_line = data[counter]
        interpolated_line_value = (next_line - previous_line) * 0.6 + previous_line
        interpolated_line.append(interpolated_line_value)
        previous_line = next_line
        if counter >= data.shape[0] - 1:
            continue_ = False
            break

        #Measure at 60% interval
        counter += 1
        next_line = data[counter]
        interpolated_line_value = (next_line - previous_line) * 0.4 + previous_line
        interpolated_line.append(interpolated_line_value)
        previous_line = next_line
        if counter >= data.shape[0] - 1:
            continue_ = False
            break

        #Measure at 80% interval
        counter += 1
        next_line = data[counter]
        interpolated_line_value = (next_line - previous_line) * 0.2 + previous_line
        interpolated_line.append(interpolated_line_value)
        if counter > data.shape[0] - 1:
            continue_ = False
            break


    interpolated_line = np.asarray(interpolated_line)
    return(interpolated_line)



if __name__ == "__main__":

    config_fn = os.path.abspath(os.getcwd()) + '/../params/dataloader_params.yaml' 
    with open(str(config_fn), 'r') as file:
	    conf = yaml.load(file, Loader=yaml.FullLoader)
    folder_directory = conf["bagfile"]

    image_ts = np.loadtxt(folder_directory + "/seq1/my_timestamps_p.txt")
    vio_states = np.loadtxt(folder_directory + "/seq1/evolving_state.txt")
    imu_meas = np.loadtxt(folder_directory + "/seq1/imu_measurements.txt")

    # Interpolated Timestamps - from 400 Hz to 500 Hz
    
    interpolated_line = interpolate_meas(image_ts)
    fn = folder_directory + "/seq1/my_timestamps_p.txt"
    np.savetxt(fn, interpolated_line)

    # Interpolated Evolving State - from 400 to 500 Hz
    interpolated_line = interpolate_meas(vio_states)
    fn = folder_directory + "/seq1/evolving_state.txt"
    np.savetxt(fn, interpolated_line)
    

    # Interpolated Imu Measurements - from 800 to 1000 Hz
    interpolated_line = interpolate_meas(imu_meas)
    fn = folder_directory + "/seq1/imu_measurements.txt"
    np.savetxt(fn, interpolated_line)
