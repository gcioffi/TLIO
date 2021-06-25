import os
import rosbag
import numpy as np
import math
import yaml
import matplotlib.pyplot as plt


def PerturbationIMUandBiases():

   
    std_values = [] #Array in which we will load bias and IMU std deviation values

    #Load yaml config file where the std of the biase and the IMU noise are defined respectively
    with open(r'/home/rpg/Desktop/param.yaml') as file:
        conf = yaml.load(file.read())
        for key, value in conf.items():
            std_values.append(value) #bias std -> [0], noise imu std -> [1]
    #Open BagFile 
    bagfile = "/home/rpg/Desktop/RaceTrajectoryIMU.bag"
    topic = "/kingfisher/ground_truth/imu"
    msgsNumber = 14594 #To be changed when a new rosbag is recorder
    header = np.empty([msgsNumber, 1]) #IMU timestamp
    orientation = [] #orientation x, y, z, w
    angular_velocity_perfect = np.empty([msgsNumber, 3]) #GT angular_velocity in x, y, z
    linear_acceleration_perfect = np.empty([msgsNumber, 3]) #GT linear_acceleration in x, y, z
    angular_velocity_IMU_noise = np.empty([msgsNumber, 3]) # GT + IMU noise
    linear_acceleration_IMU_noise = np.empty([msgsNumber, 3]) 
    angular_velocity_bias_noise = np.empty([msgsNumber, 3])  # GT + bias + noise on bias
    linear_acceleration_bias_noise = np.empty([msgsNumber, 3]) 
    angular_velocity_raw = np.empty([msgsNumber, 3]) #GT + bias + noise on bias + noise on IMU
    linear_acceleration_raw = np.empty([msgsNumber, 3]) 
    bias_acc = np.random.normal(0,std_values[0],3) #bias generation using gaussian distribution
    bias_ang = np.random.normal(0,std_values[0],3) 
    
   
    counter = -1
    with rosbag.Bag(bagfile, 'r') as bag:
        for (topic, msg, ts) in bag.read_messages(topics=str(topic)):
            
            counter = counter + 1
            header[counter, 0] = msg.header.stamp.secs 
            orientation.append([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])
            #Noise generation on IMU and bias for angular velocity and linear acceleration
            noise_IMU_ang = np.random.normal(0,std_values[1],3) #mean, std, number elements
            noise_IMU_acc = np.random.normal(0,std_values[1],3) 
            noise_bias_ang = np.random.normal(0,std_values[0],3) 
            noise_bias_acc = np.random.normal(0,std_values[0],3) 
            #Find delta_time needed to compute the noise on the biases
            if(counter > 0):
                dt_sqrt = math.sqrt(header[counter, :] - header[counter-1, :])
            else:
                dt_sqrt = 0 
            
            #Angular velocity and Linear Acceleration Perfect (what comes from the simulation)
            angular_velocity_perfect[counter, :] = np.array([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])
            linear_acceleration_perfect[counter, :] = np.array([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])
            #Angular velocity and Linear Acceleration corrupted by the IMU noise
            angular_velocity_IMU_noise[counter, :] = np.array([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z]) + noise_IMU_ang 
            linear_acceleration_IMU_noise[counter, :] = np.array([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z]) + noise_IMU_acc 
            #Angular velocity and Linear Acceleration corrupted by biases and noise on biases
            angular_velocity_bias_noise[counter, :] = np.array([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z]) + noise_bias_ang*dt_sqrt + bias_ang
            linear_acceleration_bias_noise[counter, :] = np.array([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z]) + noise_bias_acc*dt_sqrt + bias_acc
            #Angular velocity and Linear Acceleration with biases, noise on IMU and noise biases -> raw IMU 
            angular_velocity_raw[counter, :] = np.array([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z]) + noise_IMU_ang + noise_bias_ang*dt_sqrt + bias_ang
            linear_acceleration_raw[counter, :] = np.array([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z]) + noise_IMU_acc + noise_bias_acc*dt_sqrt + bias_acc
            
    bag.close()
    
    # =============================================   PLOTTING  ==========================================================

    t = header[:,0]
    # Green GT, Red Corrupted measurements

    #Plot X-angular velocity 
    plt.plot(t, angular_velocity_raw[:, 0], 'ro', t, angular_velocity_perfect[:, 0], 'go')
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


    return



if __name__ == '__main__':

    PerturbationIMUandBiases()
    


  