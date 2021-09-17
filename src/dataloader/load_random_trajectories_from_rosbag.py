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
    # example: bagfile = conf['bagfile']
    bagfile = os.path.join(conf["bagfile"], file)
    topic_imu = conf["topic_imu"]
    topic_odometry = conf["topic_odometry"]
    n_trajectories = conf["n_trajectories"]
    out_dir = conf["out_dir"]
    stdImuNoise_acc = conf["stdImuNoise_acc"]
    stdImuNoise_gyro = conf["stdImuNoise_gyro"] 
    bias_acc_min = conf["bias_acc"][0]
    bias_acc_max = conf["bias_acc"][1]
    bias_gyro_min = conf["bias_gyro"][0]
    bias_gyro_max = conf["bias_gyro"][1]
    stdBiasNoise_acc = conf["stdBiasNoise_acc"]
    stdBiasNoise_gyro = conf["stdBiasNoise_gyro"]


    print('Loading bag: %s' % bagfile)
    print('Reading topics:')
    print('- %s' % topic_imu)
    print('- %s' % topic_odometry)
    print('Saving results to: %s' % out_dir)

    
    ts_imu = [] #IMU timestamp
    ts_odom = [] #IMU timestamp
    q_wb = [] # GT orientation x, y, z, w
    p_wb = [] # GT translation x, y, z, w
    v_wb = [] #GT velocity
    w_gt = [] # GT angular_velocity in x, y, z
    a_gt = [] # GT linear_acceleration in x, y, z
    w_calib = [] # GT + IMU noise
    a_calib = [] # GT + IMU noise
    w_raw = [] #GT + bias + noise on bias + noise on IMU
    a_raw = []
    dt_sqrt = []
    first = True
    first_odom = True

    # imu measurements are interpolated to 800 Hz
    dt_interp = 0.00125 # sec

    
    with rosbag.Bag(bagfile, 'r') as bag:
       
        for (topic, msg, ts) in bag.read_messages():
            if topic[0] != '/':
                topic = "/" + topic
    
            if topic == topic_imu:
                if first:
                    #Remove preparation trajectory
                    #if msg.header.stamp.to_sec() < 1.627563898720235825e+09 or  msg.header.stamp.to_sec() > (1.62756393e+09 + 6.65): 
                    #    continue
                    dt_sqrt_ = 0
                    dt_sqrt.append(dt_sqrt_)
                    ts_imu.append(msg.header.stamp.to_sec())


                    #Angular velocity and Linear Acceleration GT (from simulation)
                    w_gt.append(np.array([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z]))
                    a_gt.append(np.array([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z]))

                    first = False

                else:
                    #if msg.header.stamp.to_sec() < 1.627563898720235825e+09 or msg.header.stamp.to_sec() > (1.62756393e+09 + 6.65): 
                    #    continue
                    prev_ts_imu = ts_imu[-1]
                    curr_ts_imu = msg.header.stamp.to_sec()
                    
                    # This is necessary in case some imu measurements are missing in the rosbag.
                    while (prev_ts_imu + dt_interp) < (curr_ts_imu - 0.0001):
                        dt = dt_interp
                        dt_sqrt_ = math.sqrt(dt) #dt_sqrt is sampled at 800 Hz
                        t = prev_ts_imu + dt

                    #IMU Interpolation: measurements at 800 Hz
                        curr_w_gt = np.array([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])
                        curr_a_gt = np.array([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])

                        prev_w_gt = w_gt[-1]
                        prev_a_gt = a_gt[-1]
                        interp_w_gt = prev_w_gt + ((curr_w_gt - prev_w_gt) / dt) * (t - prev_ts_imu)
                        interp_a_gt = prev_a_gt + ((curr_a_gt - prev_a_gt) / dt) * (t - prev_ts_imu)

                        # Append at 800 Hz: timestamps, w and a for GT and raw
                        interp_ts_imu = prev_ts_imu + dt
                        ts_imu.append(interp_ts_imu)
                        w_gt.append(interp_w_gt)
                        a_gt.append(interp_a_gt)
                        dt_sqrt.append(dt_sqrt_)

                        prev_ts_imu += dt_interp
                        
                    ts_imu.append(curr_ts_imu)
                    w_gt.append(curr_w_gt)
                    a_gt.append(curr_a_gt)
                    dt_sqrt.append(dt_sqrt_)

            if topic == topic_odometry:
                if first_odom:
                #Save GT timestamps, pose (position + orientation) and velocity from simulation -> evolving state.txt
                    # Remove offset VICON - IMU: 0.03 secs
                    ts_odom.append(msg.header.stamp.to_sec() - 0.03)
                    p_wb.append(np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]))
                    q_wb.append(np.array([msg.pose.orientation.w, msg.pose.orientation.x,msg.pose.orientation.y,msg.pose.orientation.z]))
                    v_wb.append(np.array([0,0,0]))
                    first_odom = False

                else: 
             
                    ts_odom_prev = ts_odom[-1]
                    ts_odom.append(msg.header.stamp.to_sec() - 0.03)
                    p_wb_prev = p_wb[-1]
                    p_wb.append(np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]))
                    q_wb.append(np.array([msg.pose.orientation.w, msg.pose.orientation.x,msg.pose.orientation.y,msg.pose.orientation.z]))
                    v_wb.append(np.array((p_wb[-1] - p_wb_prev)/(ts_odom[-1] - ts_odom_prev)))

    bag.close()

    # Obtain VICON velocity from VICON position and smooth the signal
    v_wb = np.array(v_wb)
    '''v_wb_mean = np.mean(v_wb, axis = 0)
    v_wb_std = np.std(v_wb, axis = 0)

    counter = -1
    while(counter != 0):

        counter = 0
        v_wb_mean = np.mean(v_wb, axis = 0)
        v_wb_std = np.std(v_wb, axis = 0)

        for i in range(v_wb.shape[0]):
            if (v_wb[i, :] > v_wb_mean + 3 * v_wb_std).any() or (v_wb[i, :] < v_wb_mean - 3 * v_wb_std).any():
                counter += 1
                v_wb[i, :] = (v_wb[i+1, :] + v_wb[i-1, :])/2
        print("Counter", counter)

    v_wb_x = savgol_filter(v_wb[:, 0], 501, 6) # window size 51, polynomial order 3
    v_wb_y = savgol_filter(v_wb[:, 1], 501, 6)
    v_wb_z = v_wb[:, 2] * 0
    #v_wb_z = savgol_filter(v_wb[:, 2], 501, 0) -> In this trajectory is simply 0
    v_wb[:, 0] = v_wb_x
    v_wb[:, 1] = v_wb_y
    v_wb[:, 2] = v_wb_z'''

    
    for i in range(n_trajectories):
        print("Creating files for trajectory:", i)

        #IMU Biases generation
        bias_acc = np.random.uniform(bias_acc_min, bias_acc_max, 3) #bias generation using uniform distribution - Lin. Accel.
        bias_w = np.random.uniform(bias_gyro_min, bias_gyro_max, 3) #bias generation using uniform distribution - Ang. Vel.
        bias_acc = np.reshape(bias_acc, (1, 3))  
        bias_w = np.reshape(bias_w, (1, 3))

        #Get np arrays
        ts_imu = np.asarray(ts_imu)
        ts_odom = np.asarray(ts_odom)
        p_wb = np.asarray(p_wb)
        q_wb = np.asarray(q_wb)
        v_wb = np.asarray(v_wb)
        dt_sqrt = np.asarray(dt_sqrt)
        dt_sqrt = np.reshape(dt_sqrt, (dt_sqrt.shape[0],1)) 

        #Find w and a as calib (GT + IMU noise) and raw values(GT + bias + IMU noise + Bias noise)
        w_calib = np.asarray(w_gt)
        a_calib = np.asarray(a_gt)
        w_raw =  np.asarray(w_gt)
        a_raw =  np.asarray(a_gt)

        # Noise generation on IMU and bias for w and a

        noise_IMU_w = np.random.normal(0,stdImuNoise_gyro,(w_calib.shape[0], w_calib.shape[1])) #mean, std, number elements
        noise_IMU_acc = np.random.normal(0,stdImuNoise_acc,(a_calib.shape[0], a_calib.shape[1]))
        noise_bias_w = np.random.normal(0,stdBiasNoise_gyro,(w_raw.shape[0], w_raw.shape[1]))
        noise_bias_acc = np.random.normal(0,stdBiasNoise_acc,(a_raw.shape[0], a_raw.shape[1]))

        w_calib = w_calib + noise_IMU_w
        a_calib = a_calib + noise_IMU_acc
                  
        w_raw = w_raw + noise_IMU_w + bias_w + noise_bias_w * dt_sqrt
        a_raw = a_raw + noise_IMU_acc + bias_acc + noise_bias_acc * dt_sqrt

        # ================================== EXPORT FILES FOR DATASET GENERATION ==================================
        

        seq_name = "seq" + str(n_trajectories*traj_analysed + i+1)
        print("seq_name", seq_name)
        seq_dir = os.path.join(out_dir, seq_name)
        if not os.path.exists(seq_dir):
            os.makedirs(seq_dir)
        
        #Generate: mytimestamps_p.txt
        fn = os.path.join(seq_dir, "my_timestamps_p.txt")
        np.savetxt(fn, ts_odom)

        #Generate: imu_measurements.txt
        ts_imu = np.reshape(ts_imu, (ts_imu.shape[0], 1))
        ts_odom = np.reshape(ts_odom, (ts_odom.shape[0], 1))

        ##Create hasVio vector
        hasVio = np.zeros((ts_imu.shape[0], 1))
        it = range(ts_odom.shape[0])
        number_hasVio = 0
        for m in it:
            if ts_odom[m] in ts_imu:
                where = np.where(ts_imu == ts_odom[m])
                hasVio[where] = 1
                number_hasVio += 1
        print("number_hasVio", number_hasVio)

        ##Structure your file
        tableIMU = np.hstack((ts_imu, a_raw, a_calib, w_raw, w_calib, hasVio))
        print(hasVio.shape)
        fn = os.path.join(seq_dir, "imu_measurements.txt")
        np.savetxt(fn, tableIMU)

        #Generate: evolving_state.txt

        tableEvolvingState = np.hstack((ts_odom, q_wb, p_wb, v_wb))
        
        fn = os.path.join(seq_dir, "evolving_state.txt")
        np.savetxt(fn, tableEvolvingState)

        # Save biases values: bias.txt
        biasesValues = np.concatenate((bias_acc,bias_w), axis=1)
       
        #check how we concatenate
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
    



