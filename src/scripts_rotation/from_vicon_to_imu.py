'''
This scripts transform the the poses of the Vicon markers to IMU poses.
Poses are in the format of evolving_state.txt (see: https://github.com/gcioffi/TLIO/tree/master)

Reference frames:
- V: (fixed) vicon frame
- M: (moving) markers frame 
- C: (moving) camera frame
- B: (moving) body (=imu) frame

Notation:
- T: 6DoF transformation matrix. Pose class
- R: rotation matrix. (3,3) numpy vector  
- p: position vector. (3,) numpy vector 
- t: time. Scalar

Inputs:
- handeye: .json file containing the transformation T_MC (obtained from https://github.com/ethz-asl/hand_eye_calibration/tree/master)
- camimu_calib: camera-imu calibration from Kalibr (format of camchain-imucam-...)
- ev_statae_fn: path to the evolving_state.txt containing the vicon markers poses

Ouput:
- aligned evolving state containing the body (= imu) poses in the vicon frame
'''

import argparse
import os

import IPython
import matplotlib.pyplot as plt
import numpy as np
from pyquaternion import Quaternion

import pose
import transformations as tf


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    #parser.add_argument("--handeye", type=str)
    #parser.add_argument("--camimu_calib", type=str)
    parser.add_argument("--ev_state_fn", type=str)
    args = parser.parse_args()

    # @TODO: load from args.handeye
    q_MC = Quaternion(np.array([0.90313326730181243, 
        -0.010070363126693096, 
        0.42923296948858486,
        -0.0028190748946359361]))
    R_MC = q_MC.rotation_matrix
    p_MC = np.array([0.070870689700682124, 
        -0.019621944091964213, 
        0.042258246141882193])
    T_MC = pose.Pose(R_MC, p_MC.reshape((3,1)))

    # @TODO: load from args.camimu_calib
    R_CB = np.array([[0.00629031, 0.62608545, -0.77972908], 
        [-0.99997865, 0.00255932, -0.00601211], 
        [-0.00176851, 0.77975026, 0.62608818]])
    p_CB = np.array([-0.03474805, 0.00664268, -0.04902435])
    T_CB = pose.Pose(R_CB, p_CB.reshape((3,1)))

    markers_states = np.loadtxt(args.ev_state_fn)

    # transform poses
    T_MB = T_MC * T_CB

    imu_states = []
    imu_ori_zyx = [] # for plotting
    markers_ori_zyx = [] # for plotting
    for s in markers_states:
        R_VM = Quaternion(np.array([s[1], s[2], s[3], s[4]])).rotation_matrix
        p_VM = np.array([s[5], s[6], s[7]])
        v_VM = np.array([s[8], s[9], s[10]])
        T_VM = pose.Pose(R_VM, p_VM.reshape(3,1))

        rz, ry, rx = tf.euler_from_matrix(R_VM, 'rzyx')
        markers_ori_zyx.append(np.array([rz, ry, rx]))

        T_VB = T_VM * T_MB
        T_VB.fix()
        q_VB = T_VB.q_wxyz()
        p_VB = T_VB.t.flatten()

        v_VB = v_VM + np.dot(R_VM, T_MB.t.flatten())

        imu_state = np.array([s[0], 
            q_VB[0], q_VB[1], q_VB[2], q_VB[3],
            p_VB[0], p_VB[1], p_VB[2], 
            v_VB[0], v_VB[1], v_VB[2]])
        imu_states.append(imu_state)
        rz, ry, rx = tf.euler_from_matrix(T_VB.R, 'rzyx')
        imu_ori_zyx.append(np.array([rz, ry, rx]))

    imu_states = np.asarray(imu_states)

    outfn = os.path.join(os.path.dirname(args.ev_state_fn), 'real_body_evolving_state.txt')
    np.savetxt(outfn, imu_states)

    # Plot position
    t = imu_states[:, 0]

    plt.figure('Position')
    plt.subplot(311)
    plt.plot(t, imu_states[:, 5], label='body')
    plt.plot(t, markers_states[:, 5], label='markers')
    plt.grid()
    plt.legend()
    plt.xlabel('t')
    plt.ylabel('x')

    plt.subplot(312)
    plt.plot(t, imu_states[:, 6], label='body')
    plt.plot(t, markers_states[:, 6], label='markers')
    plt.grid()
    plt.legend()
    plt.xlabel('t')
    plt.ylabel('y')

    plt.subplot(313)
    plt.plot(t, imu_states[:, 7], label='body')
    plt.plot(t, markers_states[:, 7], label='markers')
    plt.grid()
    plt.legend()
    plt.xlabel('t')
    plt.ylabel('z')

    # Orientations
    imu_ori_zyx = np.asarray(imu_ori_zyx)
    imu_ori_zyx = np.rad2deg(imu_ori_zyx)
    markers_ori_zyx = np.asarray(markers_ori_zyx)
    markers_ori_zyx = np.rad2deg(markers_ori_zyx)

    plt.figure('Orientations')
    plt.subplot(311)
    plt.plot(t, imu_ori_zyx[:, 0], label='body')
    plt.plot(t, markers_ori_zyx[:, 0], label='markers')
    plt.grid()
    plt.legend()
    plt.xlabel('t')
    plt.ylabel('yaw')

    plt.subplot(312)
    plt.plot(t, imu_ori_zyx[:, 1], label='body')
    plt.plot(t, markers_ori_zyx[:, 1], label='markers')
    plt.grid()
    plt.legend()
    plt.xlabel('t')
    plt.ylabel('pitch')

    plt.subplot(313)
    plt.plot(t, imu_ori_zyx[:, 2], label='body')
    plt.plot(t, markers_ori_zyx[:, 2], label='markers')
    plt.grid()
    plt.legend()
    plt.xlabel('t')
    plt.ylabel('roll')

    plt.show()

