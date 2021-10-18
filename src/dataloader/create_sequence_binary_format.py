#!/usr/bin/env python3

"""
Input (TLIO dataset format): my_timestamps_p.txt, imu_measurements.txt, evolving_state.txt
Output: data.hdf5
    - ts
    - raw accel and gyro measurements
    - vio-calibrated accel and gyro measurements
    - ground truth (vio) states (R, p, v)
    - integration rotation (with offline calibration)
    - attitude filter rotation
    - offline calibration parameters

Note: the dataset has been processed in a particular way such that the time difference between ts is almost always 1ms. 
This is for the training and testing the network. 
Image frequency is known, and IMU data is interpolated evenly between the two images.
"""

import argparse
import os

import h5py
import numpy as np
from scipy.spatial.transform import Rotation
from math_utils import mat_exp
import yaml

# @ToDo: move to a different script
def loadConfig(config_yaml):
    config = {}
    config['sequence_dir'] = config_yaml['sequence_dir']
    return config


def imu_integrate(gravity, last_state, imu_data, dt):
    """
    Given compensated IMU data and corresponding dt, propagate the previous state in the world frame
    """
    last_r = Rotation.from_rotvec(last_state[0:3])
    last_p = last_state[3:6]
    last_v = last_state[6:9]
    accel = imu_data[:3]
    omega = imu_data[3:]

    last_R = last_r.as_matrix()
    dR = mat_exp(omega * dt)

    new_R = last_R.dot(dR)
    new_v = last_v + gravity * dt + last_R.dot(accel * dt)
    new_p = (
        last_p
        + last_v * dt
        + 0.5 * gravity * dt * dt
        + 0.5 * last_R.dot(accel * dt * dt)
    )
    new_r = Rotation.from_matrix(new_R)

    return np.concatenate((new_r.as_rotvec(), new_p, new_v), axis=0)


def save_hdf5(config):
    sequence_dir = config['sequence_dir']

    gravity = np.array([0, 0, -9.81])
   
    image_ts = np.loadtxt(os.path.join(sequence_dir, "my_timestamps_p.txt"))
    imu_meas = np.loadtxt(os.path.join(sequence_dir, "imu_measurements.txt"))
    gt_states = np.loadtxt(os.path.join(sequence_dir, "evolving_state.txt"))

    # bring back ts to sec from microsec. 
    imu_meas[:,0] *= 1e-6
    gt_states[:,0] *= 1e-6

    # find initial ts where imu and vio states has same ts.
    # this is found by checking has_vio
    start_idx_imu = 0
    start_time_imu = 0.0
    for i, meas in enumerate(imu_meas):
        if meas[-1] == 1:
            start_idx_imu = i
            start_time_imu = meas[0]
            break

    start_idx_gt = 0
    for i, meas in enumerate(gt_states):
        if np.abs(meas[0] - start_time_imu) < 0.002:
            start_idx_gt = i
            break

    # get imu_data - raw and calibrated
    imu_data = imu_meas[start_idx_imu:, :]
    ts = imu_data[:, 0]
    accel_raw = imu_data[:, 1:4]
    gyro_raw = imu_data[:, 7:10]
    accel = imu_data[:, 4:7]
    gyro = imu_data[:, 10:13]

    # get state data at imu frequency by propagating states from evolving_state using imu meas
    N = imu_data.shape[0]
    state_data = np.zeros((N, 9))

    r_init = Rotation.from_quat(
        [
            gt_states[start_idx_gt, 2],
            gt_states[start_idx_gt, 3],
            gt_states[start_idx_gt, 4],
            gt_states[start_idx_gt, 1],
        ]
    )
    p_init = [
        gt_states[start_idx_gt, 5],
        gt_states[start_idx_gt, 6],
        gt_states[start_idx_gt, 7],
    ]
    v_init = [
        gt_states[start_idx_gt, 8],
        gt_states[start_idx_gt, 9],
        gt_states[start_idx_gt, 10],
    ]
    state_init = np.concatenate((r_init.as_rotvec(), p_init, v_init), axis=0)
    state_data[0, :] = state_init

    idx_gt = start_idx_gt
    for i in range(1, N):
        # get calibrated imu data for integration
        imu_data_i = np.concatenate((imu_data[i, 4:7], imu_data[i, 10:13]), axis=0)
        curr_t = imu_data[i, 0]
        past_t = imu_data[i - 1, 0]
        dt = (curr_t - past_t)  # bring ts back to sec

        # propagate
        last_state = state_data[i - 1, :]
        new_state = imu_integrate(gravity, last_state, imu_data_i, dt)
        state_data[i, :] = new_state

        # use gt if this state has gt measurement
        has_vio = int(imu_data[i, 13])

        if has_vio == 1:
            # search the corresponding gt
            for idx, meas in enumerate(gt_states[idx_gt:]):
                if np.abs(imu_data[i,0] - meas[0]) < 0.002:
                    idx_gt += idx
                    break

            assert (imu_data[i,0] - gt_states[idx_gt, 0]) < 0.002, \
            'Time mismatch between imu (=%.4f) and gt (=%.4f)' % (imu_data[i, 0], gt_states[idx_gt, 0])
            r_gt = Rotation.from_quat(
                [
                    gt_states[idx_gt, 2],
                    gt_states[idx_gt, 3],
                    gt_states[idx_gt, 4],
                    gt_states[idx_gt, 1],
                ]
            )
            p_gt = [
                gt_states[idx_gt, 5],
                gt_states[idx_gt, 6],
                gt_states[idx_gt, 7],
            ]
            v_gt = [
                gt_states[idx_gt, 8],
                gt_states[idx_gt, 9],
                gt_states[idx_gt, 10],
            ]
            gt_state = np.concatenate((r_gt.as_rotvec(), p_gt, v_gt), axis=0)
            state_data[i, :] = gt_state

    # adding timestamps in state_data
    state_data = np.concatenate(
        (np.expand_dims(imu_data[:, 0], axis=1), state_data), axis=1
    )

    # gt data
    gt_rvec = state_data[:, 1:4]
    gt_p = state_data[:, 4:7]
    gt_v = state_data[:, 7:10]
    gt_r = Rotation.from_rotvec(gt_rvec)
    gt_q = gt_r.as_quat()
    gt_q_wxyz = np.concatenate(
        [np.expand_dims(gt_q[:, 3], axis=1), gt_q[:, 0:3]], axis=1
    )
    
    # save .hdf5
    with h5py.File(os.path.join(sequence_dir, "data.hdf5"), "w") as f:
        ts_s = f.create_dataset("ts", data=ts)
        accel_dcalibrated_s = f.create_dataset("accel_dcalibrated", data=accel)
        gyro_dcalibrated_s = f.create_dataset("gyro_dcalibrated", data=gyro)
        accel_raw_s = f.create_dataset("accel_raw", data=accel_raw)
        gyro_raw_s = f.create_dataset("gyro_raw", data=gyro_raw)
        gt_q_wxyz_s = f.create_dataset("vio_q_wxyz", data=gt_q_wxyz)
        gt_p_s = f.create_dataset("vio_p", data=gt_p)
        gt_v_s = f.create_dataset("vio_v", data=gt_v)
    

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--config_fn", type=str)
    args = parser.parse_args()
    
    f = open(args.config_fn)
    config = loadConfig(yaml.load(f, Loader=yaml.FullLoader))
    f.close()

    save_hdf5(config)

