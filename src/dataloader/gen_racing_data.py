#!/usr/bin/env python3

"""
gen_fb_data.py

Input (FB dataset raw): my_timestamps_p.txt, calib_state.txt, imu_measurements.txt, evolving_state.txt, attitude.txt
Output: data.hdf5
    - ts
    - raw accel and gyro measurements
    - vio-calibrated accel and gyro measurements
    - ground truth (vio) states (R, p, v)
    - integration rotation (with offline calibration)
    - attitude filter rotation
    - offline calibration parameters

Note: the dataset has been processed in a particular way such that the time difference between ts is almost always 1ms. This is for the training and testing the network. 
Image frequency is known, and IMU data is interpolated evenly between the two images.
"""

import os
from os import path as osp
import h5py
import matplotlib.pyplot as plt
import numpy as np
import progressbar
import IPython
from numba import jit
from scipy.interpolate import interp1d
from scipy.spatial.transform import Rotation
from math_utils import mat_exp, unwrap_rpy, wrap_rpy


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


def save_hdf5(args):

    # get list of data to process
    data_dir = args.data_dir
    fn = os.path.join(data_dir, "n_sequences.txt")
    n_seq = int(np.loadtxt(fn)) 

    print("Loading %d sequences" % n_seq)
    
    name = []
    for i in range(n_seq):
        seq_name = "seq" + str(i+1)
        seq_dir = osp.join(data_dir, seq_name)
        name.append(osp.join(seq_dir))

    gravity = np.array([0, 0, -args.gravity])
    
    # save train.txt, val.txt, test.txt
    perc_train_seq = 0.8
    perc_test_seq = 0.1
    perc_val_seq = 0.1

    n_train_seq = int(perc_train_seq*n_seq)
    n_test_seq = int(perc_test_seq*n_seq)
    n_val_seq = int(perc_val_seq*n_seq)

    train_list = []
    test_list = []
    val_list = []

    #for i in progressbar.progressbar(range(n_seq), redirect_stdout=True):
    for seq_id in range(n_seq):
        datapath = name[seq_id]
       
        # start with the 20th image processed, bypass vio initialization
        image_ts = np.loadtxt(osp.join(datapath, "my_timestamps_p.txt"))
        imu_meas = np.loadtxt(osp.join(datapath, "imu_measurements.txt"))
        vio_states = np.loadtxt(osp.join(datapath, "evolving_state.txt"))
        list_idx_evolving_corresp = np.loadtxt(osp.join(datapath, "idx_evolving_corresp.txt"))
        list_idx_imu_corresp = np.loadtxt(osp.join(datapath, "idx_imu_corresp.txt"))
   

        # find initial state, start from the 21st output from vio_state
        start_t = image_ts[20]
        imu_idx = np.searchsorted(imu_meas[:, 0], start_t)
        vio_idx = np.searchsorted(vio_states[:, 0], start_t)

        # get imu_data - raw and calibrated
        print("obtain raw and vio-calibrated IMU data")
        imu_data = imu_meas[imu_idx:, :]
        ts = imu_data[:, 0] * 1e-6  
        accel_raw = imu_data[:, 1:4]  # raw
        gyro_raw = imu_data[:, 7:10]
        accel = imu_data[:, 4:7]  # calibrated with vio calibration
        gyro = imu_data[:, 10:13]

        # get state_data (same timestamps as imu_data), integrated with vio-calibrated IMU
        print("obtain vio states by integrating vio-calibrated IMU")
        N = imu_data.shape[0]
        state_data = np.zeros((N, 9))

        r_init = Rotation.from_quat(
            [
                vio_states[vio_idx, 2],
                vio_states[vio_idx, 3],
                vio_states[vio_idx, 4],
                vio_states[vio_idx, 1],
            ]
        )
        p_init = [
            vio_states[vio_idx, 5],
            vio_states[vio_idx, 6],
            vio_states[vio_idx, 7],
        ]
        v_init = [
            vio_states[vio_idx, 8],
            vio_states[vio_idx, 9],
            vio_states[vio_idx, 10],
        ]
        state_init = np.concatenate((r_init.as_rotvec(), p_init, v_init), axis=0)
        state_data[0, :] = state_init
        
        #ToDO
        counter = 0
        #for i in progressbar.progressbar(range(1, N)):

        '''
        for k in range(idx_imu_corresp.shape[0]):
            #print(idx_evolving_corresp[k])
            print(vio_states[int(idx_evolving_corresp[k]), 0], imu_meas[int(idx_imu_corresp[k]), 0])

        exit()
        '''
        print("N", N)
    
        for i in range(1,N): 
            #ToDo
            counter = counter +1  
            # get calibrated imu data for integration
            imu_data_i = np.concatenate((imu_data[i, 4:7], imu_data[i, 10:13]), axis=0)
            curr_t = imu_data[i, 0]
            past_t = imu_data[i - 1, 0]
            dt = (curr_t - past_t) * 1e-6  
                       
            last_state = state_data[i - 1, :]
            new_state = imu_integrate(gravity, last_state, imu_data_i, dt)
            state_data[i, :] = new_state

            # if this state has vio output, correct with vio
            has_vio = int(imu_data[i, 13]) 
  

            #Todo
            if has_vio == 1:
                #print("i", i)

                #vio_idx = np.searchsorted(vio_states[:, 0], imu_data[i, 0])
                #vio_idx_arr = np.where(vio_states[:, 0] == imu_data[i, 0])
                #if imu_data[i, 0] == vio_states[vio_idx_arr, 0]: print("error!")
                #vio_idx = vio_idx_arr[0][0]
                idx_in_list_imu_corresp = np.where(list_idx_imu_corresp.astype(int) == i + imu_idx) 
                idx_in_list_imu_corresp = idx_in_list_imu_corresp[0][0].astype(int)
                #print("Equal? : ", list_idx_imu_corresp[idx_in_list_imu_corresp], i)
                idx_in_list_evolving_corresp = idx_in_list_imu_corresp
                print("Equal Data? : ", vio_states[int(list_idx_evolving_corresp[idx_in_list_evolving_corresp]), 0], \
                    imu_data[i, 0])
                #print(vio_states[int(idx_evolving_corresp[k]), 0], imu_meas[int(idx_imu_corresp[k]), 0])
                vio_idx = int(list_idx_evolving_corresp[idx_in_list_evolving_corresp])
                
             
                r_vio = Rotation.from_quat(
                    [
                        vio_states[vio_idx, 2],
                        vio_states[vio_idx, 3],
                        vio_states[vio_idx, 4],
                        vio_states[vio_idx, 1],
                    ]
                )
                p_vio = [
                    vio_states[vio_idx, 5],
                    vio_states[vio_idx, 6],
                    vio_states[vio_idx, 7],
                ]
                v_vio = [
                    vio_states[vio_idx, 8],
                    vio_states[vio_idx, 9],
                    vio_states[vio_idx, 10],
                ]
                vio_state = np.concatenate((r_vio.as_rotvec(), p_vio, v_vio), axis=0)
                state_data[i, :] = vio_state 
                
        
        
    
        # adding timestamps in state_data
        state_data = np.concatenate(
            (np.expand_dims(imu_data[:, 0], axis=1), state_data), axis=1
        )

        #Todo
        # vio data
        vio_rvec = state_data[:, 1:4]
        vio_p = state_data[:, 4:7]
        vio_v = state_data[:, 7:10]
        vio_r = Rotation.from_rotvec(vio_rvec)
        vio_q = vio_r.as_quat()
        vio_q_wxyz = np.concatenate(
            [np.expand_dims(vio_q[:, 3], axis=1), vio_q[:, 0:3]], axis=1
        )

        # integrate using fixed calibration data
        print("integrate R using fixed calibrated data")
        N = ts.shape[0]
        rvec_integration = np.zeros((N, 3))
        rvec_integration[0, :] = state_data[0, 1:4]

        #for i in progressbar.progressbar(range(1, N)):
        for i in range(1,N):
            dt = ts[i] - ts[i - 1]
            last_rvec = Rotation.from_rotvec(rvec_integration[i - 1, :])
            last_R = last_rvec.as_matrix()
            omega = gyro[i, :]
            dR = mat_exp(omega * dt)
            next_R = last_R.dot(dR)
            next_r = Rotation.from_matrix(next_R)
            next_rvec = next_r.as_rotvec()
            rvec_integration[i, :] = next_rvec
        integration_r = Rotation.from_rotvec(rvec_integration)
        integration_q = integration_r.as_quat()
        integration_q_wxyz = np.concatenate(
            [np.expand_dims(integration_q[:, 3], axis=1), integration_q[:, 0:3]], axis=1
        )

        # output
        outdir = datapath
        
        # everything under the same timestamp ts
        with h5py.File(osp.join(outdir, "data.hdf5"), "w") as f:
            ts_s = f.create_dataset("ts", data=ts)
            accel_dcalibrated_s = f.create_dataset("accel_dcalibrated", data=accel)
            gyro_dcalibrated_s = f.create_dataset("gyro_dcalibrated", data=gyro)
            accel_raw_s = f.create_dataset("accel_raw", data=accel_raw)
            gyro_raw_s = f.create_dataset("gyro_raw", data=gyro_raw)
            vio_q_wxyz_s = f.create_dataset("vio_q_wxyz", data=vio_q_wxyz)
            vio_p_s = f.create_dataset("vio_p", data=vio_p)
            vio_v_s = f.create_dataset("vio_v", data=vio_v)
            print("File data.hdf5 written to " + outdir)

        seq_name = "seq" + str(seq_id+1)
        if 0 <= seq_id < n_train_seq:
            train_list.append(seq_name)
        elif n_train_seq <= seq_id < n_train_seq + n_test_seq:
            test_list.append(seq_name)
        elif seq_id >= n_train_seq + n_test_seq:
            val_list.append(seq_name)
    
    
    #Todo
    #Plotting VIO

    fig1 = plt.figure(num="prediction vs gt")
    plt.plot(vio_p[:,0], vio_p[:,1])
    plt.plot(vio_states[:,5], vio_states[:,6])
    plt.axis("equal")
    plt.legend(["Predicted", "Ground truth"])
    plt.title("2D trajectory and ATE error against time")
    fig1.savefig(osp.join(data_dir, "mygraph.png"))
    
 
    
    # Save train.txt, val.txt, test.txt
    train_fn = osp.join(data_dir, 'train.txt')
    f_txt = open(train_fn, "w")
    for fn_seq in train_list:
        f_txt.write(fn_seq)
        f_txt.write("\n")
    f_txt.close()

    test_fn = osp.join(data_dir, 'test.txt')
    f_txt = open(test_fn, "w")
    for fn_seq in test_list:
        f_txt.write(fn_seq)
        f_txt.write("\n")
    f_txt.close()

    val_fn = osp.join(data_dir, 'val.txt')
    f_txt = open(val_fn, "w")
    for fn_seq in val_list:
        f_txt.write(fn_seq)
        f_txt.write("\n")
    f_txt.close()
    

if __name__ == "__main__":

    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument("--gravity", type=float, default=9.81)
    parser.add_argument(
        "--data_dir", type=str, default="/home/rpg/Desktop/TLIO/data/Dataset"
    )

    args = parser.parse_args()

    save_hdf5(args)


#Question: imu_integrate does not take values in world frame. Is this an issue? t should be in sec?