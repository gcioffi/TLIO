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

SMALL_EPS = 0.0001
BIG_EPS = 0.002


# @ToDo: move to a different script
def loadConfig(config_yaml):
    config = {}
    config['n_seq'] = config_yaml['n_seq']
    config['dataset_dir'] = config_yaml['dataset_dir']
    config['perc_train'] = config_yaml['perc_train']
    config['perc_val'] = config_yaml['perc_val']
    config['perc_test'] = config_yaml['perc_test']
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
    dataset_dir = config['dataset_dir']
    n_seq = config['n_seq']

    # save train.txt, val.txt, test.txt
    perc_train = config['perc_train'] / 100
    perc_val = config['perc_val'] / 100
    perc_test = config['perc_test'] / 100
    n_train_seq = int(perc_train*n_seq)
    n_val_seq = int(perc_val*n_seq)
    n_test_seq = int(perc_test*n_seq)

    train_list = []
    test_list = []
    val_list = []

    gravity = np.array([0, 0, -9.81])
    
    for seq_id in range(n_seq):
        seq_name = 'seq' + str(seq_id+1)

        if 0 <= seq_id < n_train_seq:
            train_list.append(seq_name)
        elif n_train_seq <= seq_id < n_train_seq + n_val_seq:
            val_list.append(seq_name)
        elif seq_id >= n_train_seq + n_val_seq:
            test_list.append(seq_name)
        
        datapath = os.path.join(dataset_dir, seq_name)
       
        imu_meas = np.loadtxt(os.path.join(datapath, "imu_measurements.txt"))
        gt_states = np.loadtxt(os.path.join(datapath, "evolving_state.txt"))

        # bring back ts to sec from microsec. 
        imu_meas[:,0] *= 1e-6
        gt_states[:,0] *= 1e-6

        if np.abs(imu_meas[0,0] - gt_states[0,0]) > SMALL_EPS:
            if seq_id == 0:
                print('[WARNING] GT has not been sampled from the spline. Make sure this is really what you want !!')
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
                if np.abs(meas[0] - start_time_imu) < BIG_EPS:
                    start_idx_gt = i
                    break
        else:
            start_idx_imu = 0
            start_idx_gt = 0

        # get imu_data - raw and calibrated
        imu_data = imu_meas[start_idx_imu:, :]
        ts = imu_data[:, 0]
        accel_raw = imu_data[:, 1:4]
        gyro_raw = imu_data[:, 7:10]
        accel = imu_data[:, 4:7]
        gyro = imu_data[:, 10:13]

        gt_data = gt_states[start_idx_gt:, :]

        N = imu_data.shape[0]
        state_data = np.zeros((N, 9))

        if np.abs(imu_meas[0,0] - gt_states[0,0]) > SMALL_EPS:
            # get state data at imu frequency by propagating states from evolving_state using imu meas
            r_init = Rotation.from_quat([
                gt_states[start_idx_gt, 2],
                gt_states[start_idx_gt, 3],
                gt_states[start_idx_gt, 4],
                gt_states[start_idx_gt, 1],
            ])
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
                dt = (curr_t - past_t)

                # propagate
                last_state = state_data[i - 1, :]
                new_state = imu_integrate(gravity, last_state, imu_data_i, dt)
                state_data[i, :] = new_state

                # use gt if this state has gt measurement
                has_vio = int(imu_data[i, 13])

                if has_vio == 1:
                    # search the corresponding gt
                    for idx, meas in enumerate(gt_states[idx_gt:]):
                        if np.abs(imu_data[i,0] - meas[0]) < BIG_EPS:
                            idx_gt += idx
                            break

                    assert (imu_data[i,0] - gt_states[idx_gt, 0]) < BIG_EPS, \
                    'Time mismatch between imu (=%.4f) and gt (=%.4f)' % (imu_data[i, 0], gt_states[idx_gt, 0])
                    r_gt = Rotation.from_quat([
                            gt_states[idx_gt, 2],
                            gt_states[idx_gt, 3],
                            gt_states[idx_gt, 4],
                            gt_states[idx_gt, 1],
                        ])
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
        else:
            assert imu_data.shape[0] == gt_data.shape[0], \
            'len imu (=%d) != len gt (=%d)' % (imu_data.shape[0], gt_data.shape[0])
            for i in range(gt_data.shape[0]):
                r = Rotation.from_quat([
                    gt_states[i, 2], 
                    gt_states[i, 3], 
                    gt_states[i, 4], 
                    gt_states[i, 1] 
                    ])
                p = [
                    gt_states[i, 5],
                    gt_states[i, 6],
                    gt_states[i, 7]
                ]
                v = [
                    gt_states[i, 8],
                    gt_states[i, 9],
                    gt_states[i, 10]
                ]
                state_i = np.concatenate((r.as_rotvec(), p, v), axis=0)
                state_data[i, :] = state_i
    
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
        with h5py.File(os.path.join(datapath, "data.hdf5"), "w") as f:
            ts_s = f.create_dataset("ts", data=ts)
            accel_dcalibrated_s = f.create_dataset("accel_dcalibrated", data=accel)
            gyro_dcalibrated_s = f.create_dataset("gyro_dcalibrated", data=gyro)
            accel_raw_s = f.create_dataset("accel_raw", data=accel_raw)
            gyro_raw_s = f.create_dataset("gyro_raw", data=gyro_raw)
            gt_q_wxyz_s = f.create_dataset("vio_q_wxyz", data=gt_q_wxyz)
            gt_p_s = f.create_dataset("vio_p", data=gt_p)
            gt_v_s = f.create_dataset("vio_v", data=gt_v)

        if seq_id % 10 == 0:
            print('Processed seq: %d' % seq_id)
    
    # Save train.txt, val.txt, test.txt
    train_fn = os.path.join(dataset_dir, 'train.txt')
    f_txt = open(train_fn, "w")
    for fn_seq in train_list:
        f_txt.write(fn_seq)
        f_txt.write("\n")
    f_txt.close()

    val_fn = os.path.join(dataset_dir, 'val.txt')
    f_txt = open(val_fn, "w")
    for fn_seq in val_list:
        f_txt.write(fn_seq)
        f_txt.write("\n")
    f_txt.close()

    test_fn = os.path.join(dataset_dir, 'test.txt')
    f_txt = open(test_fn, "w")
    for fn_seq in test_list:
        f_txt.write(fn_seq)
        f_txt.write("\n")
    f_txt.close()
    

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--config_fn", type=str)
    args = parser.parse_args()
    
    f = open(args.config_fn)
    config = loadConfig(yaml.load(f, Loader=yaml.FullLoader))
    f.close()

    save_hdf5(config)

