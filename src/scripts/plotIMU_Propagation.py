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
from scipy.spatial.transform import Rotation
import rosbag


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


def plotPosXY(p_odom, raw_imu_p, calib_imu_p):
    plt.plot(p_odom[:, 0], p_odom[:, 1], label='odom')
    plt.plot(raw_imu_p[:, 0], raw_imu_p[:, 1], label='raw imu')
    plt.plot(calib_imu_p[:, 0], calib_imu_p[:, 1], label='calib imu')
    plt.grid()
    plt.legend()
    plt.xlabel('x')
    plt.ylabel('y')


def propagate(p0, R0, v0, acc, gyr, ts, ba, bg):
    g = np.array([0, 0, -9.81])

    p = []
    p.append(p0)
    
    R_prev = R0
    R_curr = R_prev

    v_prev = v0
    v_curr = v_prev

    for i in np.arange(1, len(ts)):
        dt = ts[i] - ts[i-1]
        w = gyr[i, :]
        a = acc[i, :]

        dtheta = (w - bg) * dt
        dRd = mat_exp(dtheta)
        R_curr = R_prev @ dRd
        dv_w = R_prev @ (a - ba) * dt
        dp_w = 0.5 * dv_w * dt
        gdt = g * dt
        gdt22 = 0.5 * gdt * dt
        v_curr = v_prev + dv_w + gdt
        p_curr = p[-1] + v_prev * dt + dp_w + gdt22

        R_prev = R_curr
        v_prev = v_curr
        p.append(p_curr)

    return p


def propagateWithGtOrientation(p0, q, gt_ts, v0, acc, gyr, ts, ba, bg):
    print("in propagation function")
    g = np.array([0, 0, -9.81])

    p = []
    p.append(p0)

    v_prev = v0
    v_curr = v_prev

    for i in np.arange(1, len(ts)):
        if ts[i] in gt_ts:
            gt_idx = np.where(gt_ts == ts[i])[0][0]

            dt = ts[i] - ts[i-1]
            w = gyr[i, :]
            a = acc[i, :]

            R = q[gt_idx].rotation_matrix
            dv_w = R @ (a - ba) * dt
            dp_w = 0.5 * dv_w * dt
            gdt = g * dt
            gdt22 = 0.5 * gdt * dt
            v_curr = v_prev + dv_w + gdt
            p_curr = p[-1] + v_prev * dt + dp_w + gdt22

            v_prev = v_curr
            p.append(p_curr)

    return p


def run(args):
    # Load data
    with open(args.data_list) as f:
        data_list = [s.strip() for s in f.readlines() if len(s.strip()) > 0]
       
    ## use only first sequence
    data_dir = os.path.join(args.data_dir, data_list[1])
    print("Analysis", data_list[1])

    ## vio
    gt_states = np.loadtxt(os.path.join(data_dir, "evolving_state.txt"))
    gt_ts = gt_states[:, 0] # sec
    gt_p = gt_states[:, 5:8]
    gt_v = gt_states[:, 8:11]
    gt_rq = gt_states[:, 1:5]
    '''vio_r = Rotation.from_quat(
        np.concatenate([gt_rq[:, 1:4], np.expand_dims(gt_rq[:, 0], axis=1)], axis=1) )'''

    interp_gt_ts = []
    interp_gt_ts.append(gt_ts[0]) #First ts
    interp_gt_rq = []
    interp_gt_rq.append(Quaternion(gt_rq[0])) #Initial rotation
    for i in range(gt_rq.shape[0]-1): #Obtain rotation in quaternion from GT and interpolate
        q0 = Quaternion(gt_rq[i])
        q1 = Quaternion(gt_rq[i+1])
        q_interp = Quaternion.slerp(q0, q1, amount=0.5)
        interp_gt_rq.append(q_interp)
        interp_gt_rq.append(q1)
        dt = gt_ts[i+1] - gt_ts[i]
        interp_gt_ts.append(gt_ts[i] + dt*0.5)
        interp_gt_ts.append(gt_ts[i+1])

    
    ## biases - original version
    #biases_path = os.path.join(data_dir, "Biases.txt")
    #values = []
    #fhand = open(biases_path)
    #counter_line = -1
    #for line in fhand:
    #    counter_line = counter_line + 1 
    #    if counter_line == 0: continue
    #    line = line.rstrip()
    #    contents = line.split(':')
    #    values.append(float(contents[1]))
    #gt_acc_bias = np.array([values[0], values[1], values[2]])
    #gt_gyro_bias = np.array([values[3], values[4], values[5]])

    ## biases - If debug using no noise and no biases
    gt_acc_bias = np.array([0, 0, 0])
    gt_gyro_bias = np.array([0, 0, 0])
    ## imu
    imu_data = np.loadtxt(os.path.join(data_dir, "imu_measurements.txt")) 
    ts = imu_data[:, 0]  # s
    accel_raw = imu_data[:, 1:4]  # raw
    gyro_raw = imu_data[:, 7:10]
    accel = imu_data[:, 4:7]  # calibrated with vio calibration
    gyro = imu_data[:, 10:13]
    
    ## propagate imu measurements
    q0 = Quaternion(gt_rq[0, :])

    raw_imu_prop_p = propagate(gt_p[0, :], q0.rotation_matrix, gt_v[0, :], accel_raw, gyro_raw, ts, gt_acc_bias, gt_gyro_bias)
    calib_imu_prop_p = propagate(gt_p[0, :], q0.rotation_matrix, gt_v[0, :], accel, gyro, ts, np.zeros((3,)), np.zeros((3,)))

    # use gt orientation
    #raw_imu_prop_p = propagateWithGtOrientation(gt_p[0, :], interp_gt_rq, \
    #    interp_gt_ts, gt_v[0, :], accel_raw, gyro_raw, ts, gt_acc_bias, gt_gyro_bias)
    #calib_imu_prop_p = propagateWithGtOrientation(gt_p[0, :], interp_gt_rq, \
    #    interp_gt_ts, gt_v[0, :], accel, gyro, ts, np.zeros((3,)), np.zeros((3,)))

    fig = plt.figure(0)
    plotPosXY(gt_p, np.asarray(raw_imu_prop_p), np.asarray(calib_imu_prop_p))

    # IPython.embed()

    plt.show()
    

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("--data_dir", type=str) # for me, data/Lemniscate_No_Noise
    parser.add_argument("--data_list", type=str, default="test.txt") # strange.txt analizzare 7 & 14 di train
    args = parser.parse_args()
    print(vars(args))
    run(args)

# Terminal: /Desktop/TLIO/src/scripts
# Command to launch: python3 plotIMU_Propagation.py --data_dir ../../data/Dataset --data_list ../../data/Dataset/test.txt