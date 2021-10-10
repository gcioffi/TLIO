import argparse
#import IPython
import os
import rosbag
import numpy as np
import math
import yaml
import matplotlib.pyplot as plt
import re
import pandas as pd
from scipy.signal import savgol_filter

'''
Reference frames:
- W: fixed world frame
- B: drone body frame
- I: imu frame same as B
- T: 6 DoF pose
- R: rotation matrix
- t: 3D translation
'''


def to_keys(text):
    #return int(re.split('.bag', text)[0])
    return(int(re.search('[0-9][0-9]*', text).group(0)))


def perturbationIMUandBiases(config_fn, file, conf, traj_analysed, rosbags_num, all_biases):

    #Load yaml config file where the std deviation of bias, bias noise and IMU noise are defined
    bagfile = os.path.join(conf["bagfile"], file)
    topic_imu = conf["topic_imu"]
    topic_odometry = conf["topic_odometry"]
    n_trajectories = conf["n_trajectories"]
    out_dir = conf["out_dir"]

    bias_acc_x = conf["bias_acc"][0]
    bias_acc_y = conf["bias_acc"][1]
    bias_acc_z = conf["bias_acc"][2]
    bias_gyro_x = conf["bias_gyro"][0]
    bias_gyro_y = conf["bias_gyro"][1]
    bias_gyro_z = conf["bias_gyro"][2]

    print('Loading bag: %s' % bagfile)
    print('Reading topics:')
    print('- %s' % topic_imu)
    print('- %s' % topic_odometry)
    print('Saving results to: %s' % out_dir)

    ts_imu = [] 
    w_raw = []
    a_raw = []
    w_calib = []
    a_calib = []

    ts_odom = []
    q_wb = [] 
    p_wb = [] 
    v_wb = [] 

    dt_sqrt = []

    first_imu = True

    with rosbag.Bag(bagfile, 'r') as bag:       
        for (topic, msg, ts) in bag.read_messages():
            if topic[0] != '/':
                topic = "/" + topic
    
            if topic == topic_imu:

                ts_imu.append(msg.header.stamp.to_sec())
                w_raw.append(np.array([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z]))
                a_raw.append(np.array([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z]))
                if first_imu:
                    dt_sqrt.append(0) 
                else:
                    dt_sqrt.append(ts_imu[-1] - ts_imu[-2])
                first_imu = False
                

            if topic == topic_odometry: # 400 Hz
                ts_odom.append(msg.header.stamp.to_sec())
                p_wb.append(np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]))
                q_wb.append(np.array([msg.pose.orientation.w, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z]))
                v_wb.append(np.array([0,0,0])) # Filled later

    bag.close()

    v_wb = np.array(v_wb)
    
    for i in range(n_trajectories):
        print("Creating files for trajectory:", i)
        # Odometry
        ts_odom = np.asarray(ts_odom)
        ts_odom = np.reshape(ts_odom, (ts_odom.shape[0], 1))
        p_wb = np.asarray(p_wb)
        q_wb = np.asarray(q_wb)
        v_wb = np.asarray(v_wb)

        dt_sqrt = np.asarray(dt_sqrt)
        dt_sqrt = np.reshape(dt_sqrt, (dt_sqrt.shape[0],1)) 

        # IMU
        ts_imu = np.asarray(ts_imu)
        ts_imu = np.reshape(ts_imu, (ts_imu.shape[0], 1))
        w_raw =  np.asarray(w_raw)
        a_raw =  np.asarray(a_raw)   

        # Find calibrated w and a as calib (RAW - bias)
        w_calib = np.asarray(w_raw)
        a_calib = np.asarray(a_raw)      
        bias_acc = np.array([bias_acc_x, bias_acc_y, bias_acc_z])
        bias_w = np.array([bias_gyro_x, bias_gyro_y, bias_gyro_z])
        w_calib = w_raw - bias_w
        a_calib = a_raw - bias_acc 

        # ============== EXPORT FILES FOR DATASET GENERATION ==================================
        
        seq_name = "seq" + str(n_trajectories*traj_analysed + i+1)
        print("seq_name", seq_name)
        seq_dir = os.path.join(out_dir, seq_name)
        if not os.path.exists(seq_dir):
            os.makedirs(seq_dir)
        
        # Generate: mytimestamps_p.txt
        fn = os.path.join(seq_dir, "my_timestamps_p.txt")
        np.savetxt(fn, ts_odom)

        # Generate: imu_measurements.txt
        ## Create hasVio vector -> filled in later stage.
        hasVio = np.zeros((ts_imu.shape[0], 1))
        ## Structure your file
        tableIMU = np.hstack((ts_imu, a_raw, a_calib, w_raw, w_calib, hasVio))
        fn = os.path.join(seq_dir, "imu_measurements.txt")
        np.savetxt(fn, tableIMU)

        # Generate: evolving_state.txt
        tableEvolvingState = np.hstack((ts_odom, q_wb, p_wb, v_wb))
        fn = os.path.join(seq_dir, "evolving_state.txt")
        np.savetxt(fn, tableEvolvingState)

        # Save biases values: bias.txt
        biasesValues = np.hstack((bias_acc, bias_w))       
        # Check how we concatenate
        all_biases.append(np.asarray(biasesValues))
        targetsBiases  = np.array(["Bias_acc_x", "Bias_acc_y", "Bias_acc_z", "Bias_w_x", "Bias_w_y", "Bias_w_z"])
        biasesValues = np.array(biasesValues)
        ab = np.zeros(targetsBiases.size, dtype=[('Targets', 'U256'), ('Biases', float)])
        ab['Targets'] = targetsBiases
        ab['Biases'] = biasesValues
        fn = os.path.join(seq_dir, "Biases.txt")
        np.savetxt(fn, ab, fmt="%-10s : %-10.3f", header="Bias type, Value")

        
    if traj_analysed == rosbags_num - 1:
   
        fn = os.path.join(out_dir, "n_sequences.txt")
        np.savetxt(fn, np.array([n_trajectories * rosbags_num], dtype=int), fmt='%d')
        all_biases = np.array(all_biases)
        all_biases = np.reshape(all_biases, (n_trajectories * rosbags_num, 6))
        targetsBiases  = np.array(["Bias_acc_x", "Bias_acc_y", "Bias_acc_z", "Bias_w_x", "Bias_w_y", "Bias_w_z"])
        fn = os.path.join(out_dir, "all_biases.txt")
        np.savetxt(fn, all_biases, header = "Bias_acc_x, Bias_acc_y, Bias_acc_z, Bias_w_x, Bias_w_y, Bias_w_z")
        
    return 


if __name__ == '__main__':

    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=str, default='/home/rpg/Desktop/TLIO/src/params/dataloader_params.yaml')
    parser.add_argument("--continue_from", type=int, default=0) # write index of bag = number_bag_in_the_name - 1 (of the bag you want to read from)
    args = parser.parse_args()
    config_fn = args.config
    
    with open(str(config_fn), 'r') as file:
	    conf = yaml.load(file, Loader=yaml.FullLoader)

    fpath = os.path.abspath(conf["bagfile"])
    traj_analysed = args.continue_from
    all_biases = [] 

    for(dirpath, dirnames, filenames) in os.walk(fpath):
        rosbags_num = len(filenames)

        for file in sorted(filenames):
            if sorted(filenames).index(file) == traj_analysed:
                print("Analysis Bag", file)
                perturbationIMUandBiases(config_fn, file, conf, traj_analysed, rosbags_num, all_biases)
                traj_analysed = traj_analysed + 1
            else:
                continue
        break
    



