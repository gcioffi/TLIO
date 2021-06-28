import argparse
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


def plotImu(times, meas, title):
    plt.subplot(311)
    plt.plot(times, meas[:, 0], label='x')
    plt.grid()
    plt.legend()
    plt.xlabel('t')
    plt.ylabel('x')
    plt.title(title)
    
    plt.subplot(312)
    plt.plot(times, meas[:, 1], label='y')
    plt.grid()
    plt.legend()
    plt.xlabel('t')
    plt.ylabel('y')

    plt.subplot(313)
    plt.plot(times, meas[:, 2], label='z')
    plt.grid()
    plt.legend()
    plt.xlabel('t')
    plt.ylabel('z')


def PerturbationIMUandBiases(config_fn):

    #Dictionary with standard deviation values
    std_values = {'stdImuNoise_acc': 0, 'stdImuNoise_gyro': 0, 'stdBias_acc': 0, 'stdBias_gyro': 0,'stdBiasNoise_acc': 0, 'stdBiasNoise_gyro': 0} 

    #Load yaml config file where the std deviation of bias, bias noise and IMU noise are defined
    with open(config_fn) as file:
        conf = yaml.load(file.read())
        for key, value in conf.items():
            if key == 'bagfile':
                bagfile = value
            if key == 'topic_imu':
                topic_imu = value
            if key == 'topic_odometry':
                topic_odometry = value
            else:
                if key == 'stdImuNoise_acc':
                    #std_values['a'] = value 
                    stdImuNoise_acc = value
                if key == 'stdImuNoise_gyro':
                    stdImuNoise_gyro = value
                if key == 'stdBias_acc':
                    stdBias_acc = value
                if key == 'stdBias_gyro':
                    stdBias_gyro = value
                if key == 'stdBiasNoise_acc':
                    stdBiasNoise_acc = value
                if key == 'stdBiasNoise_gyro':
                    stdBiasNoise_gyro = value
            
    
    print('Loaded bag: %s' % bagfile)
    print('Reading topics:')
    print('- %s' % topic_imu)
    print('- %s' % topic_odometry)

    #Open BagFile
    # @ ToDO: add groundtruth odometry
    ts_imu = [] #IMU timestamp
    ts_odom = [] #IMU timestamp
    q_wb = [] # GT orientation x, y, z, w
    p_wb = [] # GT translation x, y, z, w
    v_wb = [] #GT velocity
    angular_velocity_perfect = [] #GT angular_velocity in x, y, z
    linear_acceleration_perfect = [] #GT linear_acceleration in x, y, z
    angular_velocity_IMU_noise = [] # GT + IMU noise
    linear_acceleration_IMU_noise = []
    #angular_velocity_bias_noise = []  # GT + bias + noise on bias
    #linear_acceleration_bias_noise = []
    angular_velocity_raw = [] #GT + bias + noise on bias + noise on IMU
    linear_acceleration_raw = []
    bias_acc = np.random.normal(0,stdBias_acc,3) #bias generation using gaussian distribution - Lin. Accel.
    bias_ang = np.random.normal(0,stdBias_gyro,3) #bias generation using gaussian distribution - Ang. Vel. 
    
   
    first = True
    with rosbag.Bag(bagfile, 'r') as bag:
        for (topic, msg, ts) in bag.read_messages():
            if topic == topic_imu:
                #IMU Interpolation at 1000 Hz -> IMU_measurements.txt

                # Noise generation on IMU and bias for angular velocity and linear acceleration
                noise_IMU_ang = np.random.normal(0,stdImuNoise_gyro,3) #mean, std, number elements
                noise_IMU_acc = np.random.normal(0,stdImuNoise_acc,3) 
                noise_bias_ang = np.random.normal(0,stdBiasNoise_gyro,3) 
                noise_bias_acc = np.random.normal(0,stdBiasNoise_acc,3) 
                # Find delta_time needed to compute the noise on the biases
                if first:
                    dt_sqrt = 0
                    ts_imu.append(msg.header.stamp.to_sec())

                    #Angular velocity and Linear Acceleration PERFECT (from simulation)
                    angular_velocity_perfect.append(np.array([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z]))
                    linear_acceleration_perfect.append(np.array([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z]))
                    #Angular velocity and Linear Acceleration corrupted by the IMU NOISE
                    angular_velocity_IMU_noise.append(np.array([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z]) + noise_IMU_ang)
                    linear_acceleration_IMU_noise.append(np.array([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z]) + noise_IMU_acc)
                    #Angular velocity and Linear Acceleration corrupted by BIASES and BIASES NOISE
                    '''angular_velocity_bias_noise.append(np.array([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z]) + noise_bias_ang*dt_sqrt + bias_ang)
                    linear_acceleration_bias_noise.append(np.array([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z]) + noise_bias_acc*dt_sqrt + bias_acc)'''
                    #Angular velocity and Linear Acceleration with biases, noise on IMU and noise biases -> RAW IMU 
                    angular_velocity_raw.append(np.array([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z]) + noise_IMU_ang + noise_bias_ang*dt_sqrt + bias_ang)
                    linear_acceleration_raw.append(np.array([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z]) + noise_IMU_acc + noise_bias_acc*dt_sqrt + bias_acc)

                    first = False

                else:
                    curr_ts_imu = msg.header.stamp.to_sec()
                    prev_ts_imu = ts_imu[-1]

                    dt_sqrt = math.sqrt(curr_ts_imu - prev_ts_imu)
                    dt = curr_ts_imu - prev_ts_imu
                    t = prev_ts_imu + dt / 2.0
                    
                    #GT IMU Interpolation: measurements will be now at 1000 Hz
                                      
                    curr_w_gt = np.array([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])
                    curr_a_gt = np.array([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])
                    
                    prev_w_gt = angular_velocity_perfect[-1]
                    prev_a_gt = linear_acceleration_perfect[-1]

                    interp_w_gt = prev_w_gt + ((curr_w_gt - prev_w_gt) / dt) * (t - prev_ts_imu)
                    interp_a_gt = prev_a_gt + ((curr_a_gt - prev_a_gt) / dt) * (t - prev_ts_imu)

                    #Raw IMU Interpolation: measurements will be now at 1000 Hz

                    curr_w_raw = np.array([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z]) + noise_IMU_ang + noise_bias_ang*dt_sqrt + bias_ang
                    curr_a_raw = np.array([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z]) + noise_IMU_acc + noise_bias_acc*dt_sqrt + bias_acc
                    
                    prev_w_raw = angular_velocity_raw[-1]
                    prev_a_raw = linear_acceleration_raw[-1]

                    interp_w_raw = prev_w_raw + ((curr_w_raw - prev_w_raw) / dt) * (t - prev_ts_imu)
                    interp_a_raw = prev_a_raw + ((curr_a_raw - prev_a_raw) / dt) * (t - prev_ts_imu)


                    # Append at 1000 Hz: timestamps, w and a for GT and raw
                    interp_ts_imu = prev_ts_imu + dt / 2.0 
                    ts_imu.append(interp_ts_imu)
                    ts_imu.append(curr_ts_imu)

                    angular_velocity_perfect.append(interp_w_gt)
                    angular_velocity_perfect.append(curr_w_gt)

                    linear_acceleration_perfect.append(interp_a_gt)
                    linear_acceleration_perfect.append(curr_a_gt)

                    angular_velocity_raw.append(interp_w_raw)
                    angular_velocity_raw.append(curr_w_raw)

                    linear_acceleration_raw.append(interp_a_raw)
                    linear_acceleration_raw.append(curr_a_raw)

            if topic == topic_odometry:
                #Save GT timestamps, pose (position + orientation) and velocity from simulation -> evolving state.txt
                ts_odom.append(msg.header.stamp.to_sec())
                p_wb.append(np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z]))
                q_wb.append(np.array([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w]))
                v_wb.append(np.array([msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z]))
  
    bag.close()

    #Transform lists to arrays
    ts_imu = np.asarray(ts_imu)
    ts_odom = np.asarray(ts_odom)
    angular_velocity_perfect = np.asarray(angular_velocity_perfect)
    linear_acceleration_perfect = np.asarray(linear_acceleration_perfect)
    angular_velocity_raw =  np.asarray(angular_velocity_raw)
    linear_acceleration_raw =  np.asarray(linear_acceleration_raw)
    p_wb = np.asarray(p_wb)
    q_wb = np.asarray(q_wb)
    v_wb = np.asarray(v_wb)

    

    # ================================== EXPORT FILES FOR DATASET GENERATION ==================================
    
    #Generate: mytimestamps_p.txt

    with open("my_timestamps_p.txt", "w") as txt_file:
        for line in ts_odom.tolist():
            txt_file.write(str(line)) 
            txt_file.write("\n") 

    #Generate: imu_measurements.txt

    ts_imu_row, col = linear_acceleration_raw.shape 
    ts_imu = np.reshape(ts_imu, (ts_imu_row, 1))

    ts_odom_row, col = q_wb.shape
    ts_odom = np.reshape(ts_odom, (ts_odom_row, 1))

        #Create hasVio vector
    hasVio = np.zeros((ts_imu_row, 1))
    it = range(ts_odom_row)
    for m in it:
        if ts_odom[m] in ts_imu:
            where = np.where(ts_imu == ts_odom[m])
            hasVio[where] = 1

        #Structure your file
    tableIMU = np.hstack((ts_imu, linear_acceleration_raw, linear_acceleration_perfect, angular_velocity_raw, angular_velocity_perfect, hasVio))
       
    file = open("imu_measurements.txt", "w")
    for row in tableIMU:      
        line = str(row).lstrip('[').rstrip(']')
        line += (f'\n')
        line += (f'\n')
        file.write(line)
    file.close()

    #Generate: evolving_state.txt

    tableEvolvingState = np.hstack((ts_odom, q_wb, p_wb, v_wb))
    file = open("evolving_state.txt", "w")
    for row in tableEvolvingState:    
        line = str(row).lstrip('[').rstrip(']')
        line += (f'\n')
        line += (f'\n')
        file.write(line)
    file.close()

    # =============================================   PLOTTING  ===========================hasVio===============================

    #fig = plt.figure()
    #plotImu(ts_imu, linear_acceleration_perfect, 'Gt acc. measurements')
    #plotImu(ts_imu, linear_acceleration_raw, 'Raw acc. measurements')

    #Plot X-angular velocity 
    '''plt.plot(t, angular_velocity_raw[:, 0], 'ro', t, angular_velocity_perfect[:, 0], 'go')
    plt.xlabel('t') 
    plt.ylabel('Angular Velocity: x') 
    plt.title("GT vs. Raw")
    plt.show()
    plt.plot(t, angular_velocity_IMU_noise[:, 0], 'ro', t, angular_velocity_perfect[:, 0], 'go')
    plt.xlabel('t') 
    plt.ylabel('Angular Velocity: x') 
    plt.title("GT vs. IMU noise")
    plt.show()
    plt.plot(t, angular_velocity_bias_noise[:, 0], 'ro', t, angular_velocity_perfect[:, 0], 'go')
    plt.xlabel('t') 
    plt.ylabel('Angular Velocity: x') 
    plt.title("GT vs. Bias and its noise")
    plt.show()

    #Plot Y-angular velocity 
    plt.plot(t, angular_velocity_raw[:, 1], 'ro', t, angular_velocity_perfect[:, 1], 'go')
    plt.xlabel('t') 
    plt.ylabel('Angular Velocity: y') 
    plt.title("GT vs. Raw")
    plt.show()
    plt.plot(t, angular_velocity_IMU_noise[:, 1], 'ro', t, angular_velocity_perfect[:, 1], 'go')
    plt.xlabel('t') 
    plt.ylabel('Angular Velocity: y') 
    plt.title("GT vs. IMU noise")
    plt.show()
    plt.plot(t, angular_velocity_bias_noise[:, 1], 'ro', t, angular_velocity_perfect[:, 1], 'go')
    plt.xlabel('t') 
    plt.ylabel('Angular Velocity: y') 
    plt.title("GT vs. Bias and its noise")
    plt.show()

    #Plot Z-angular velocity 
    plt.plot(t, angular_velocity_raw[:, 2], 'ro', t, angular_velocity_perfect[:, 2], 'go')
    plt.xlabel('t') 
    plt.ylabel('Angular Velocity: z') 
    plt.title("GT vs. Raw")
    plt.show()
    plt.plot(t, angular_velocity_IMU_noise[:, 2], 'ro', t, angular_velocity_perfect[:, 2], 'go')
    plt.xlabel('t') 
    plt.ylabel('Angular Velocity: z') 
    plt.title("GT vs. IMU noise")
    plt.show()
    plt.plot(t, angular_velocity_bias_noise[:, 2], 'ro', t, angular_velocity_perfect[:, 2], 'go')
    plt.xlabel('t') 
    plt.ylabel('Angular Velocity: z') 
    plt.title("GT vs. Bias and its noise")
    plt.show()

    #Plot X-linear acceleration
    plt.plot(t, linear_acceleration_raw[:, 0], 'ro', t, linear_acceleration_perfect[:, 0], 'go')
    plt.xlabel('t') 
    plt.ylabel('Linear Acceleration: x') 
    plt.title("GT vs. Raw")
    plt.show()
    plt.plot(t, linear_acceleration_IMU_noise[:, 0], 'ro', t, linear_acceleration_perfect[:, 0], 'go')
    plt.xlabel('t') 
    plt.ylabel('Linear Acceleration: x') 
    plt.title("GT vs. IMU noise")
    plt.show()
    plt.plot(t, linear_acceleration_bias_noise[:, 0], 'ro', t, linear_acceleration_perfect[:, 0], 'go')
    plt.xlabel('t') 
    plt.ylabel('Linear Acceleration: x') 
    plt.title("GT vs. Bias and its noise")
    plt.show()

    #Plot Y-linear acceleration
    plt.plot(t, linear_acceleration_raw[:, 1], 'ro', t, linear_acceleration_perfect[:, 1], 'go')
    plt.xlabel('t') 
    plt.ylabel('Linear Acceleration: y') 
    plt.title("GT vs. Raw")
    plt.show()
    plt.plot(t, linear_acceleration_IMU_noise[:, 1], 'ro', t, linear_acceleration_perfect[:, 1], 'go')
    plt.xlabel('t') 
    plt.ylabel('Linear Acceleration: y') 
    plt.title("GT vs. IMU noise")
    plt.show()
    plt.plot(t, linear_acceleration_bias_noise[:, 1], 'ro', t, linear_acceleration_perfect[:, 1], 'go')
    plt.xlabel('t') 
    plt.ylabel('Linear Acceleration: y') 
    plt.title("GT vs. Bias and its noise")
    plt.show()

    #Plot Z-linear acceleration
    plt.plot(t, linear_acceleration_raw[:, 2], 'ro', t, linear_acceleration_perfect[:, 2], 'go')
    plt.xlabel('t') 
    plt.ylabel('Linear Acceleration: z') 
    plt.title("GT vs. Raw")
    plt.show()
    plt.plot(t, linear_acceleration_IMU_noise[:, 2], 'ro', t, linear_acceleration_perfect[:, 2], 'go')
    plt.xlabel('t') 
    plt.ylabel('Linear Acceleration: z') 
    plt.title("GT vs. IMU noise")
    plt.show()
    plt.plot(t, linear_acceleration_bias_noise[:, 2], 'ro', t, linear_acceleration_perfect[:, 2], 'go')
    plt.xlabel('t') 
    plt.ylabel('Linear Acceleration: z') 
    plt.title("GT vs. Bias and its noise")
    plt.show()
    '''

    


    return



if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=str, default='/home/rpg/Desktop/param.yaml')
    args = parser.parse_args()
    config_fn = args.config
    PerturbationIMUandBiases(config_fn)
    
