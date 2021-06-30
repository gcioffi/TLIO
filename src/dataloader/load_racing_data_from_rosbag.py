import argparse
#import IPython
import os
import rosbag
import numpy as np
import math
import yaml
import matplotlib.pyplot as plt


'''
Reference frames:
- W: fixed world frame
- B: drone body frame
- I: imu frame same as B
- T: 6 DoF pose
- R: rotation matrix
- t: 3D translation
'''


def perturbationIMUandBiases(config_fn):

    #Dictionary with standard deviation values
    std_values = {'stdImuNoise_acc': 0, 'stdImuNoise_gyro': 0, 'stdBias_acc': 0, 'stdBias_gyro': 0,'stdBiasNoise_acc': 0, 'stdBiasNoise_gyro': 0}

    #Load yaml config file where the std deviation of bias, bias noise and IMU noise are defined
    with open(config_fn) as file:
        conf = yaml.load(file.read())

        # example
        # bagfile = conf['bagfile']
        for key, value in conf.items():
            if key == 'bagfile':
                bagfile = value
            if key == 'topic_imu':
                topic_imu = value
            if key == 'topic_odometry':
                topic_odometry = value
            # Num of sequence
            if key == 'n_trajectories':
                n_trajectories = value
            if key == 'out_dir':
                out_dir = value
            else:
                if key == 'stdImuNoise_acc':
                    stdImuNoise_acc = value
                if key == 'stdImuNoise_gyro':
                    stdImuNoise_gyro = value
                if key == 'bias_acc':
                    bias_acc_min = value[0]
                    bias_acc_max = value[1]
                if key == 'bias_gyro':
                    bias_gyro_min = value[0]
                    bias_gyro_max = value[1]
                if key == 'stdBiasNoise_acc':
                    stdBiasNoise_acc = value
                if key == 'stdBiasNoise_gyro':
                    stdBiasNoise_gyro = value

    print('Loading bag: %s' % bagfile)
    print('Reading topics:')
    print('- %s' % topic_imu)
    print('- %s' % topic_odometry)
    print('Saving results to: %s' % out_dir)

    all_biases = []
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


    with rosbag.Bag(bagfile, 'r') as bag:
        for (topic, msg, ts) in bag.read_messages():
            if topic == topic_imu:
                #IMU Interpolation at 1000 Hz -> IMU_measurements.txt

                # Noise generation on IMU and bias for angular velocity and linear acceleration

                if first:
                    dt_sqrt = 0
                    dt_sqrt.append(dt_sqrt)
                    ts_imu.append(msg.header.stamp.to_sec())

                    #Angular velocity and Linear Acceleration GT (from simulation)
                    w_gt.append(np.array([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z]))
                    a_gt.append(np.array([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z]))

                    first = False

                else:
                    curr_ts_imu = msg.header.stamp.to_sec()
                    prev_ts_imu = ts_imu[-1]
                    dt_sqrt = math.sqrt(curr_ts_imu - prev_ts_imu)
                    dt_sqrt.append(dt_sqrt)
                    dt = curr_ts_imu - prev_ts_imu
                    t = prev_ts_imu + dt / 2.0

                    #Calib IMU Interpolation: measurements will be now at 1000 Hz
                    curr_w_gt = np.array([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])
                    curr_a_gt = np.array([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])

                    prev_w_gt = w_gt[-1]
                    prev_a_gt = a_gt[-1]

                    interp_w_gt = prev_w_gt + ((curr_w_gt - prev_w_gt) / dt) * (t - prev_ts_imu)
                    interp_a_gt = prev_a_gt + ((curr_a_gt - prev_a_gt) / dt) * (t - prev_ts_imu)


                    # Append at 1000 Hz: timestamps, w and a for GT and raw
                    interp_ts_imu = prev_ts_imu + dt / 2.0
                    ts_imu.append(interp_ts_imu)
                    ts_imu.append(curr_ts_imu)

                    w_gt.append(interp_w_gt)
                    a_gt.append(curr_a_gt)


            if topic == topic_odometry:
                #Save GT timestamps, pose (position + orientation) and velocity from simulation -> evolving state.txt
                ts_odom.append(msg.header.stamp.to_sec())
                p_wb.append(np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z]))
                q_wb.append(np.array([msg.pose.pose.orientation.w, msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z]))
                v_wb.append(np.array([msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z]))

    bag.close()



    for i in range(n_trajectories):
        print("Creating file with trajectory:", i)

        bias_acc = np.random.uniform(bias_acc_min, bias_acc_max, 3) #bias generation using uniform distribution - Lin. Accel.
        bias_w = np.random.uniform(bias_gyro_min, bias_gyro_max, 3) #bias generation using uniform distribution - Ang. Vel.

        w_calib = np.asarray(w_gt)
        a_calib = np.asarray(a_gt)
        w_raw =  np.asarray(w_gt)
        a_raw =  np.asarray(a_gt)
        ts_imu = np.asarray(ts_imu)
        ts_odom = np.asarray(ts_odom)
        p_wb = np.asarray(p_wb)
        q_wb = np.asarray(q_wb)
        v_wb = np.asarray(v_wb)
        dt_sqrt = np.asarray(dt_sqrt)

        noise_IMU_w = np.random.normal(0,stdImuNoise_gyro,(w_calib.shape[0], w_calib.shape[1])) #mean, std, number elements
        noise_IMU_acc = np.random.normal(0,stdImuNoise_acc,(a_calib.shape[0], a_calib.shape[1]))
        noise_bias_w = np.random.normal(0,stdBiasNoise_gyro,(w_raw.shape[0], w_raw.shape[1]))
        noise_bias_acc = np.random.normal(0,stdBiasNoise_acc,(a_raw.shape[0], a_raw.shape[1]))

        w_calib = w_calib + noise_IMU_w
        a_calib = a_calib + noise_IMU_acc
        w_raw = w_raw + noise_IMU_w + bias_w + noise_bias_w* dt_sqrt
        a_raw = a_raw + noise_IMU_acc + bias_acc + noise_bias_acc* dt_sqrt


        #Transform lists to arrays

        # ================================== EXPORT FILES FOR DATASET GENERATION ==================================
        seq_name = "seq" + str(i+1)
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
        for m in it:
            if ts_odom[m] in ts_imu:
                where = np.where(ts_imu == ts_odom[m])
                hasVio[where] = 1

        ##Structure your file
        tableIMU = np.hstack((ts_imu, a_raw, a_calib, w_raw, w_calib, hasVio))

        fn = os.path.join(seq_dir, "imu_measurements.txt")
        np.savetxt(fn, tableIMU)

        #Generate: evolving_state.txt

        tableEvolvingState = np.hstack((ts_odom, q_wb, p_wb, v_wb))
        fn = os.path.join(seq_dir, "evolving_state.txt")
        np.savetxt(fn, tableEvolvingState)

        # Save biases values: bias.txt

        biasesValues = np.concatenate((bias_acc,bias_w), axis=0)
        all_biases.append(np.asarray(biasesValues))
        targetsBiases  = np.array(["Bias_acc_x", "Bias_acc_y", "Bias_acc_z", "Bias_w_x", "Bias_w_y", "Bias_w_z"])
        biasesValues = np.array(biasesValues)
        ab = np.zeros(targetsBiases.size, dtype=[('Targets', 'U256'), ('Biases', float)])
        ab['Targets'] = targetsBiases
        ab['Biases'] = biasesValues
        fn = os.path.join(seq_dir, "Biases.txt")
        np.savetxt(fn, ab, fmt="%-10s : %-10.3f", header="Bias type, Value")


    # Save others
    fn = os.path.join(out_dir, "n_sequences.txt")
    np.savetxt(fn, np.array([n_trajectories], dtype=int), fmt='%d')

    #ToDo
    all_biases = np.array(all_biases)
    targetsBiases  = np.array(["Bias_acc_x", "Bias_acc_y", "Bias_acc_z", "Bias_w_x", "Bias_w_y", "Bias_w_z"])
    fn = os.path.join(out_dir, "all_biases.txt")
    np.savetxt(fn, all_biases, header = "Bias_acc_x, Bias_acc_y, Bias_acc_z, Bias_w_x, Bias_w_y, Bias_w_z")



    # IPython.embed()

    return


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=str, default='/home/rpg/Desktop/TLIO/src/params/dataloader_params.yaml')
    args = parser.parse_args()
    config_fn = args.config
    perturbationIMUandBiases(config_fn)



'''import argparse
import IPython
import os
import rosbag
import numpy as np
import math
import yaml
import matplotlib.pyplot as plt

  
Reference frames:
- W: fixed world frame
- B: drone body frame
- I: imu frame same as B
- T: 6 DoF pose
- R: rotation matrix
- t: 3D translation
  


def perturbationIMUandBiases(config_fn):

    #Dictionary with standard deviation values
    std_values = {'stdImuNoise_acc': 0, 'stdImuNoise_gyro': 0, 'stdBias_acc': 0, 'stdBias_gyro': 0,'stdBiasNoise_acc': 0, 'stdBiasNoise_gyro': 0} 

    #Load yaml config file where the std deviation of bias, bias noise and IMU noise are defined
    with open(config_fn) as file:
        conf = yaml.load(file.read())
        
        # example
        # bagfile = conf['bagfile'] 
        for key, value in conf.items():
            if key == 'bagfile':
                bagfile = value
            if key == 'topic_imu':
                topic_imu = value
            if key == 'topic_odometry':
                topic_odometry = value
            # Num of sequence
            if key == 'n_trajectories':
                n_trajectories = value
            if key == 'out_dir':
                out_dir = value 
            else:
                if key == 'stdImuNoise_acc':
                    stdImuNoise_acc = value
                if key == 'stdImuNoise_gyro':
                    stdImuNoise_gyro = value
                if key == 'bias_acc':
                    bias_acc_min = value[0]
                    bias_acc_max = value[1]
                if key == 'bias_gyro':
                    bias_gyro_min = value[0]
                    bias_gyro_max = value[1]
                if key == 'stdBiasNoise_acc':
                    stdBiasNoise_acc = value
                if key == 'stdBiasNoise_gyro':
                    stdBiasNoise_gyro = value
            
    print('Loaded bag: %s' % bagfile)
    print('Reading topics:')
    print('- %s' % topic_imu)
    print('- %s' % topic_odometry)
    print('Saving results to: %s' % out_dir)

    all_biases = []

    for i in range(n_trajectories):
        print("Creating file with trajectory:", i)

        bias_acc = np.random.uniform(bias_acc_min, bias_acc_max, 3) #bias generation using uniform distribution - Lin. Accel.
        bias_w = np.random.uniform(bias_gyro_min, bias_gyro_max, 3) #bias generation using uniform distribution - Ang. Vel.

        #Open BagFile
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
        
    
    
        first = True
        with rosbag.Bag(bagfile, 'r') as bag:
            for (topic, msg, ts) in bag.read_messages():
                if topic == topic_imu:
                    #IMU Interpolation at 1000 Hz -> IMU_measurements.txt

                    # Noise generation on IMU and bias for angular velocity and linear acceleration
                    noise_IMU_w = np.random.normal(0,stdImuNoise_gyro,3) #mean, std, number elements
                    noise_IMU_acc = np.random.normal(0,stdImuNoise_acc,3) 
                    noise_bias_w = np.random.normal(0,stdBiasNoise_gyro,3) 
                    noise_bias_acc = np.random.normal(0,stdBiasNoise_acc,3)

                    if first:
                        dt_sqrt = 0
                        ts_imu.append(msg.header.stamp.to_sec())

                        #Angular velocity and Linear Acceleration GT (from simulation)
                        w_gt.append(np.array([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z]))
                        a_gt.append(np.array([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z]))
                        #Angular velocity and Linear Acceleration corrupted by the IMU NOISE
                        w_calib.append(np.array([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z]) + noise_IMU_w)
                        a_calib.append(np.array([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z]) + noise_IMU_acc)
                        #Angular velocity and Linear Acceleration with biases, noise on IMU and noise biases -> RAW IMU 
                        w_raw.append(np.array([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z]) + noise_IMU_w + noise_bias_w*dt_sqrt + bias_w)
                        a_raw.append(np.array([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z]) + noise_IMU_acc + noise_bias_acc*dt_sqrt + bias_acc)

                        first = False

                    else:
                        curr_ts_imu = msg.header.stamp.to_sec()
                        prev_ts_imu = ts_imu[-1]

                        dt_sqrt = math.sqrt(curr_ts_imu - prev_ts_imu)
                        dt = curr_ts_imu - prev_ts_imu
                        t = prev_ts_imu + dt / 2.0
                        
                        #Calib IMU Interpolation: measurements will be now at 1000 Hz
                        curr_w_calib = np.array([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z]) + noise_IMU_w
                        curr_a_calib = np.array([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z]) + noise_IMU_acc
                        
                        prev_w_calib = w_calib[-1]
                        prev_a_calib = a_calib[-1]

                        interp_w_calib = prev_w_calib + ((curr_w_calib - prev_w_calib) / dt) * (t - prev_ts_imu)
                        interp_a_calib = prev_a_calib + ((curr_a_calib - prev_a_calib) / dt) * (t - prev_ts_imu)

                        #Raw IMU Interpolation: measurements will be now at 1000 Hz

                        curr_w_raw = np.array([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z]) + noise_IMU_w + noise_bias_w*dt_sqrt + bias_w
                        curr_a_raw = np.array([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z]) + noise_IMU_acc + noise_bias_acc*dt_sqrt + bias_acc
                        
                        prev_w_raw = w_raw[-1]
                        prev_a_raw = a_raw[-1]

                        interp_w_raw = prev_w_raw + ((curr_w_raw - prev_w_raw) / dt) * (t - prev_ts_imu)
                        interp_a_raw = prev_a_raw + ((curr_a_raw - prev_a_raw) / dt) * (t - prev_ts_imu)

                        # Append at 1000 Hz: timestamps, w and a for GT and raw
                        interp_ts_imu = prev_ts_imu + dt / 2.0 
                        ts_imu.append(interp_ts_imu)
                        ts_imu.append(curr_ts_imu)

                        w_calib.append(interp_w_calib)
                        w_calib.append(curr_w_calib)

                        a_calib.append(interp_a_calib)
                        a_calib.append(curr_a_calib)

                        w_raw.append(interp_w_raw)
                        w_raw.append(curr_w_raw)

                        a_raw.append(interp_a_raw)
                        a_raw.append(curr_a_raw)

                        # just for debugging
                        w_gt.append(np.array([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z]))
                        a_gt.append(np.array([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z]))

                if topic == topic_odometry:
                    #Save GT timestamps, pose (position + orientation) and velocity from simulation -> evolving state.txt
                    ts_odom.append(msg.header.stamp.to_sec())
                    p_wb.append(np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z]))
                    q_wb.append(np.array([msg.pose.pose.orientation.w, msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z]))
                    v_wb.append(np.array([msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z]))
    
        bag.close()

        #Transform lists to arrays
        ts_imu = np.asarray(ts_imu)
        ts_odom = np.asarray(ts_odom)
        w_calib = np.asarray(w_calib)
        a_calib = np.asarray(a_calib)
        w_raw =  np.asarray(w_raw)
        a_raw =  np.asarray(a_raw)
        p_wb = np.asarray(p_wb)
        q_wb = np.asarray(q_wb)
        v_wb = np.asarray(v_wb)

        
        # ================================== EXPORT FILES FOR DATASET GENERATION ==================================
        seq_name = "seq" + str(i+1)
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
        for m in it:
            if ts_odom[m] in ts_imu:
                where = np.where(ts_imu == ts_odom[m])
                hasVio[where] = 1

        ##Structure your file
        tableIMU = np.hstack((ts_imu, a_raw, a_calib, w_raw, w_calib, hasVio))
        
        fn = os.path.join(seq_dir, "imu_measurements.txt")
        np.savetxt(fn, tableIMU)

        #Generate: evolving_state.txt

        tableEvolvingState = np.hstack((ts_odom, q_wb, p_wb, v_wb))
        fn = os.path.join(seq_dir, "evolving_state.txt")
        np.savetxt(fn, tableEvolvingState)

        # Save biases values: bias.txt
        
        biasesValues = np.concatenate((bias_acc,bias_w), axis=0)  
        all_biases.append(np.asarray(biasesValues))
        targetsBiases  = np.array(["Bias_acc_x", "Bias_acc_y", "Bias_acc_z", "Bias_w_x", "Bias_w_y", "Bias_w_z"])
        biasesValues = np.array(biasesValues)
        ab = np.zeros(targetsBiases.size, dtype=[('Targets', 'U256'), ('Biases', float)])
        ab['Targets'] = targetsBiases
        ab['Biases'] = biasesValues
        fn = os.path.join(seq_dir, "Biases.txt")
        np.savetxt(fn, ab, fmt="%-10s : %-10.3f", header="Bias type, Value")    


    # Save others
    fn = os.path.join(out_dir, "n_sequences.txt")
    np.savetxt(fn, np.array([n_trajectories], dtype=int), fmt='%d')

    #ToDo
    all_biases = np.array(all_biases)
    targetsBiases  = np.array(["Bias_acc_x", "Bias_acc_y", "Bias_acc_z", "Bias_w_x", "Bias_w_y", "Bias_w_z"])
    fn = os.path.join(out_dir, "all_biases.txt")
    np.savetxt(fn, all_biases, header = "Bias_acc_x, Bias_acc_y, Bias_acc_z, Bias_w_x, Bias_w_y, Bias_w_z")



    # IPython.embed()

    return


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=str, default='/home/rpg/Desktop/TLIO/src/params/dataloader_params.yaml')
    args = parser.parse_args()
    config_fn = args.config
    perturbationIMUandBiases(config_fn)
    
'''