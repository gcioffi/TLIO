'''
This script plots the gt trajectory against the trajectory obtained by integrating 
the gt imu measurements and the noisy imu measurements.
'''

import argparse
import os

import IPython
import matplotlib.pyplot as plt
import numpy as np
from pyquaternion import Quaternion
import rosbag

# from tlio.plots import plotImuCalibVsRaw
from tlio.utils import readDataloaderParams

ACC_NOISE_STD = 0.1
GYRO_NOISE_STD = 0.01

# util functions
def hat(v):
        v = v.flatten()
        R = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
        return R


def mat_exp(omega):
    if len(omega) != 3:
        raise ValueError("tangent vector must have length 3")
    angle = np.linalg.norm(omega)

    # Near phi==0, use first order Taylor expansion
    if angle < 1e-10:
        return np.identity(3) + hat(omega)

    axis = omega / angle
    s = np.sin(angle)
    c = np.cos(angle)

    return c * np.identity(3) + (1 - c) * np.outer(axis, axis) + s * hat(axis)


def loadFromRosbag(configs):
    bagfile = configs['bagfile']
    topic_imu = configs['topic_imu']
    topic_odometry = configs['topic_odometry']

    imu = {}
    ts_imu  = []
    acc = []
    gyro = []

    odometry = {}
    ts_odom = []
    pos_odom = []
    q_odom = []
    v_odom = []

    with rosbag.Bag(bagfile, 'r') as bag:
        for (topic, msg, ts) in bag.read_messages():
            if topic == topic_odometry:
                ts_odom.append(msg.header.stamp.to_sec())
                pos_odom.append(np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z]))
                q_odom.append(np.array([msg.pose.pose.orientation.w, msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z]))
                v_odom.append(np.array([msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z]))

        for (topic, msg, ts) in bag.read_messages():
            # read only imu measurements that have a corresponding odometry meas.
            if topic == topic_imu:
                t = msg.header.stamp.to_sec()
                if t in ts_odom:
                    ts_imu.append(msg.header.stamp.to_sec())
                    gyro.append(np.array([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z]))
                    acc.append(np.array([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z]))

    imu['ts'] = ts_imu
    imu['acc'] = acc
    imu['gyro'] = gyro

    odometry['ts'] = ts_odom
    odometry['pos'] = pos_odom
    odometry['ori'] = q_odom
    odometry['vel'] = v_odom

    return imu, odometry


def integrateIMU(imu, odometry):
    ts_imu = imu['ts']
    acc = imu['acc']
    gyro = imu['gyro']

    dt = ts_imu[1] - ts_imu[0]
    g = np.array([0.0, 0.0, -9.81])
    W_v = np.array([0., 0., 0.])

    ts_odom = odometry['ts']
    pos_odom = odometry['pos']
    ori_odom = odometry['ori']
    
    pos_WB = []
    pos_WB.append(pos_odom[0])

    R_WB = []
    R_WB.append(Quaternion(ori_odom[0]).rotation_matrix)

    for i in range(len(ts_imu) - 1):
        assert ts_imu[i] == ts_odom[i], \
        'ts_imu[i] (= %.6f) != ts_odom[i] (= %.6f)' % (ts_imu[i], ts_odom[i])

        # use gt rotation
        '''q_WB_0 = Quaternion(ori_odom[i])
        q_WB_1 = Quaternion(ori_odom[i+1])
        q_WB = Quaternion.slerp(q_WB_0, q_WB_1, amount=0.5)
        R_WB = q_WB.rotation_matrix'''

        w = (gyro[i+1] + gyro[i]) * 0.5
        dtheta = w * dt
        dRd = mat_exp(dtheta)
        R_curr = np.dot(R_WB[-1], dRd)

        B_a = (acc[i+1] + acc[i])  * 0.5
        W_a = np.dot(R_WB[-1], B_a) + g
        R_WB.append(R_curr)

        # vel
        W_v += W_a * dt

        # p
        p_WB = pos_WB[-1] + W_v * dt + 0.5 * W_a * dt * dt
        pos_WB.append(p_WB)

    return pos_WB


def plotPosXY(p_odom, p_imu, p_imu_noisy):
    plt.plot(p_odom[:, 0], p_odom[:, 1], label='odom')
    plt.plot(p_imu[:, 0], p_imu[:, 1], label='imu')
    plt.plot(p_imu_noisy[:, 0], p_imu_noisy[:, 1], label='noisy imu')
    plt.grid()
    plt.legend()
    plt.xlabel('x')
    plt.ylabel('y')


def run(configs):
    imu, odometry = loadFromRosbag(configs)

    p_odom = odometry['pos']
    p_imu = integrateIMU(imu, odometry)

    # get noisy imu
    acc_noisy = []
    gyro_noisy = []

    for i in range(len(imu['ts'])):
        acc_noisy.append(imu['acc'][i] + np.random.normal(0, ACC_NOISE_STD, (3, )))
        gyro_noisy.append(imu['gyro'][i] + np.random.normal(0, GYRO_NOISE_STD, (3, )))

    imu_noisy = {}
    imu_noisy['ts'] = imu['ts']
    imu_noisy['acc'] = acc_noisy
    imu_noisy['gyro'] = gyro_noisy

    p_imu_noisy = integrateIMU(imu_noisy, odometry)

    # plot
    p_odom = np.asarray(p_odom)
    p_imu = np.asarray(p_imu)
    p_imu_noisy = np.asarray(p_imu_noisy)

    fig = plt.figure(0)
    plotPosXY(p_odom, p_imu, p_imu_noisy)

    #IPython.embed()
    
    plt.show()
    

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("--config_fn", type=str)
    args = parser.parse_args()
    configs = readDataloaderParams(args.config_fn)

    run(configs)

