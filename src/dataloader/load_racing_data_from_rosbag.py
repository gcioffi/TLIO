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

    # @ToDo: make a dictionary
    # std_values = {} #Array in which we will load bias and IMU std deviation values
    std_values = [] #Array in which we will load bias and IMU std deviation values

    #Load yaml config file where the std of the biase and the IMU noise are defined respectively
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
                #if key == 'a':
                    #std_values['a'] = value 
                std_values.append(value) #bias std -> [0], noise imu std -> [1]
    
    print('Loaded bag: %s' % bagfile)
    print('Reading topics:')
    print('- %s' % topic_imu)
    print('- %s' % topic_odometry)

    #Open BagFile
    ts_imu = [] #IMU timestamp
    # @ ToDO: add groundtruth odometry
    ts_odom = [] #IMU timestamp
    q_wb = [] # orientation x, y, z, w
    p_wb = [] # translation x, y, z, w
    angular_velocity_perfect = [] #GT angular_velocity in x, y, z
    linear_acceleration_perfect = [] #GT linear_acceleration in x, y, z
    angular_velocity_IMU_noise = [] # GT + IMU noise
    linear_acceleration_IMU_noise = []
    angular_velocity_bias_noise = []  # GT + bias + noise on bias
    linear_acceleration_bias_noise = []
    angular_velocity_raw = [] #GT + bias + noise on bias + noise on IMU
    linear_acceleration_raw = []
    bias_acc = np.random.normal(0,std_values[0],3) #bias generation using gaussian distribution
    bias_ang = np.random.normal(0,std_values[0],3) 
    
   
    first = True
    with rosbag.Bag(bagfile, 'r') as bag:
        for (topic, msg, ts) in bag.read_messages():
            if topic == topic_imu:
                # Noise generation on IMU and bias for angular velocity and linear acceleration
                noise_IMU_ang = np.random.normal(0,std_values[1],3) #mean, std, number elements
                noise_IMU_acc = np.random.normal(0,std_values[1],3) 
                noise_bias_ang = np.random.normal(0,std_values[0],3) 
                noise_bias_acc = np.random.normal(0,std_values[0],3) 
                # Find delta_time needed to compute the noise on the biases
                if first:
                    dt_sqrt = 0

                    ts_imu.append(msg.header.stamp.to_sec())

                    #Angular velocity and Linear Acceleration Perfect (what comes from the simulation)
                    angular_velocity_perfect.append(np.array([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z]))
                    linear_acceleration_perfect.append(np.array([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z]))
                    #Angular velocity and Linear Acceleration corrupted by the IMU noise
                    angular_velocity_IMU_noise.append(np.array([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z]) + noise_IMU_ang )
                    linear_acceleration_IMU_noise.append(np.array([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z]) + noise_IMU_acc )
                    #Angular velocity and Linear Acceleration corrupted by biases and noise on biases
                    '''angular_velocity_bias_noise.append(np.array([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z]) + noise_bias_ang*dt_sqrt + bias_ang)
                    linear_acceleration_bias_noise.append(np.array([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z]) + noise_bias_acc*dt_sqrt + bias_acc)'''
                    #Angular velocity and Linear Acceleration with biases, noise on IMU and noise biases -> raw IMU 
                    angular_velocity_raw.append(np.array([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z]) + noise_IMU_ang + noise_bias_ang*dt_sqrt + bias_ang)
                    linear_acceleration_raw.append(np.array([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z]) + noise_IMU_acc + noise_bias_acc*dt_sqrt + bias_acc)

                    first = False

                else:
                    curr_ts_imu = msg.header.stamp.to_sec()
                    prev_ts_imu = ts_imu[-1]

                    dt_sqrt = math.sqrt(curr_ts_imu - prev_ts_imu)

                    curr_w_gt = np.array([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])
                    curr_a_gt = np.array([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])
                    
                    prev_w_gt = angular_velocity_perfect[-1]
                    prev_a_gt = linear_acceleration_perfect[-1]

                    # interpolate measurements
                    dt = curr_ts_imu - prev_ts_imu
                    t = prev_ts_imu + dt / 2.0
                    
                    interp_w_gt = prev_w_gt + ((curr_w_gt - prev_w_gt) / dt) * (t - prev_ts_imu)
                    interp_a_gt = prev_a_gt + ((curr_a_gt - prev_a_gt) / dt) * (t - prev_ts_imu)

                    # save
                    interp_ts_imu = prev_ts_imu + dt / 2.0 
                    ts_imu.append(interp_ts_imu)
                    ts_imu.append(curr_ts_imu)

                    angular_velocity_perfect.append(interp_w_gt)
                    angular_velocity_perfect.append(curr_w_gt)

                    linear_acceleration_perfect.append(interp_a_gt)
                    linear_acceleration_perfect.append(curr_a_gt)

            if topic == topic_odometry:
                ts_odom.append(msg.header.stamp.to_sec())
            
            
    bag.close()

    ts_imu = np.asarray(ts_imu)
    ts_odom = np.asarray(ts_odom)

    linear_acceleration_perfect = np.asarray(linear_acceleration_perfect)

    print(ts_imu[0:50])
    print(ts_odom[0:50])
    
    # =============================================   PLOTTING  ==========================================================
    # Green GT, Red Corrupted measurements

    fig = plt.figure()
    plotImu(ts_imu, linear_acceleration_perfect, 'Gt acc. measurements')

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
    plt.title("GT vs. Bias and its noise")'''


    plt.show()


    return



if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=str, default='/home/rpg/Desktop/param.yaml')
    args = parser.parse_args()

    config_fn = args.config

    PerturbationIMUandBiases(config_fn)
    
