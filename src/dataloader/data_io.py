from os import path as osp

import h5py
import numpy as np
from scipy.interpolate import interp1d
from scipy.spatial.transform import Rotation
from utils.logging import logging
from utils.math_utils import unwrap_rpy, wrap_rpy
import numpy.matlib


class DataIO:
    def __init__(self):
        # raw dataset - ts in us
        self.ts_all = None
        self.acc_all = None
        self.gyr_all = None
        self.dataset_size = None
        self.init_ts = None
        self.R_init = np.eye(3)
        # vio data
        self.vio_ts = None
        self.vio_p = None
        self.vio_v = None
        self.vio_eul = None
        self.vio_R = None
        self.vio_rq = None
        self.vio_ba = None
        self.vio_bg = None
        # attitude filter data
        self.filter_ts = None
        self.filter_eul = None

    def load_all(self, dataset, args):
        """
        load timestamps, accel and gyro data from dataset
        """
        with h5py.File(osp.join(args.root_dir, dataset, "data.hdf5"), "r") as f:
            # we save ts in .hdf5 as 1e-6 * sec
            ts_all = np.copy(f["ts"]) * 1e12 # usec
            acc_all = np.copy(f["accel_raw"])
            gyr_all = np.copy(f["gyro_raw"])
        if args.start_from_ts is not None:
            idx_start = np.where(ts_all >= args.start_from_ts)[0][0]
        else:
            # Start from first measurements
            # (differently from original code where idx_start = 50)
            idx_start = 0
        self.ts_all = ts_all[idx_start:]
        self.acc_all = acc_all[idx_start:, :]
        self.gyr_all = gyr_all[idx_start:, :]
        self.dataset_size = self.ts_all.shape[0]
        self.init_ts = self.ts_all[0]

    def load_filter(self, dataset, args):
        """
        load rotation from attitude filter and its timestamps
        """
        attitude_filter_path = osp.join(args.root_dir, dataset, "attitude.txt")
        attitudes = np.loadtxt(attitude_filter_path, delimiter=",", skiprows=3)
        self.filter_ts = attitudes[:, 0] * 1e-6
        filter_r = Rotation.from_quat(
            np.concatenate(
                [attitudes[:, 2:5], np.expand_dims(attitudes[:, 1], axis=1)], axis=1
            )
        )
        R_filter = filter_r.as_matrix()
        R_wf = np.array([[1, 0, 0], [0, 0, -1], [0, 1, 0]])
        R_filter = np.matmul(R_wf, R_filter)
        filter_r = Rotation.from_matrix(R_filter)
        self.filter_eul = filter_r.as_euler("xyz", degrees=True)

    def load_vio(self, dataset, args):
        """
        load ts, p, q, v from vio states, load ba and bg from calibration states
        """
        logging.info(
            "loading vio states from "
            + osp.join(args.root_dir, dataset, "evolving_state.txt")
        )
        vio_states = np.loadtxt(
            osp.join(args.root_dir, dataset, "evolving_state.txt") )
        # ts in evolving_state.txt are secs
        self.vio_ts = vio_states[:, 0]
        self.vio_p = vio_states[:, 5:8]
        self.vio_v = vio_states[:, 8:11]
        self.vio_rq = vio_states[:, 1:5]
        vio_r = Rotation.from_quat(
            np.concatenate(
                [self.vio_rq[:, 1:4], np.expand_dims(self.vio_rq[:, 0], axis=1)], axis=1
            )
        )
        self.vio_eul = vio_r.as_euler("xyz", degrees=True)
        self.vio_R = vio_r.as_matrix()

        biases_fn = osp.join(args.root_dir, dataset, "Biases.txt")
        values = []
        fhand = open(biases_fn)

        counter_line = -1
        for line in fhand:
            counter_line = counter_line + 1 
            if counter_line == 0: continue
            line = line.rstrip()
            contents = line.split(':')
            values.append(float(contents[1]))

        accelBias = np.array([values[0], values[1], values[2]]).reshape((3, 1))
        gyroBias = np.array([values[3], values[4], values[5]]).reshape((3, 1))

        vio_my_ts = np.loadtxt(
            osp.join(args.root_dir, dataset, "my_timestamps_p.txt") )    
        # ts in my_timestamps_p.txt are in sec

        self.vio_calib_ts = self.ts_all * 1e-6 # sec
        
        num_my_ts = self.vio_calib_ts.shape[0]
        self.vio_ba = np.matlib.repmat(accelBias, 1, num_my_ts).T
        self.vio_bg = np.matlib.repmat(gyroBias, 1, num_my_ts).T

    def load_sim_data(self, args):
        """
        This loads simulation data from an imu.csv file containing
        perfect imu data.
        """
        logging.info("loading simulation data from " + args.sim_data_path)
        sim_data = np.loadtxt(
            args.sim_data_path,
            delimiter=",",
            usecols=(0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16),
        )
        ts_all = sim_data[:, 0]
        vio_p = sim_data[:, 1:4]
        vio_rq = sim_data[:, 4:8]
        acc_all = sim_data[:, 8:11]
        vio_v = sim_data[:, 11:14]
        gyr_all = sim_data[:, 14:17]

        # add sim noise and bias
        if args.add_sim_imu_noise:
            wa = args.sim_sigma_na * np.random.normal(0, 1, acc_all.shape)
            wg = args.sim_sigma_ng * np.random.normal(0, 1, gyr_all.shape)
            acc_all = acc_all + wa
            gyr_all = gyr_all + wg

            sim_ba = np.array([0.3, -0.2, 0.4])
            sim_bg = np.array([0.0005, 0.002, -0.001])
            acc_all = acc_all + sim_ba
            gyr_all = gyr_all + sim_bg

        if args.start_from_ts is not None:
            idx_start = np.where(ts_all >= args.start_from_ts * 1e-6)[0][0]
        else:
            idx_start = 50

        self.ts_all = ts_all[idx_start:] * 1e6
        self.acc_all = 0.5 * (acc_all[idx_start:, :] + acc_all[idx_start - 1 : -1, :])
        self.gyr_all = 0.5 * (gyr_all[idx_start:, :] + gyr_all[idx_start - 1 : -1, :])
        # self.acc_all = acc_all[idx_start:,:]
        # self.gyr_all = gyr_all[idx_start:,:]
        self.vio_ts = ts_all[idx_start - 1 :]
        self.vio_p = vio_p[idx_start - 1 :, :]
        self.vio_v = vio_v[idx_start - 1 :, :]
        self.vio_rq = vio_rq[idx_start - 1 :, :]

        vio_r = Rotation.from_quat(self.vio_rq)
        self.vio_eul = vio_r.as_euler("xyz", degrees=True)
        self.vio_R = vio_r.as_matrix()

        self.dataset_size = self.ts_all.shape[0]
        self.init_ts = self.ts_all[0]

    def get_datai(self, idx):
        ts = self.ts_all[idx] * 1e-6  # s
        acc = self.acc_all[idx, :].reshape((3, 1))
        gyr = self.gyr_all[idx, :].reshape((3, 1))
        return ts, acc, gyr

    def get_meas_from_vio(self, ts_oldest_state, ts_end):
        """
        helper function This extracts a fake measurement from vio,
        can be used for debug to bypass the network
        """
        # to sec
        ts_oldest_state *= 1e-6
        ts_end *= 1e-6

        if ts_end > self.vio_ts[-1]:
            return None, None, False

        # obtain vio_Ri for rotating to relative frame
        idx_left = np.where(self.vio_ts < ts_oldest_state)[0][-1]
        idx_right = np.where(self.vio_ts > ts_oldest_state)[0][0]
        interp_vio_ts = self.vio_ts[idx_left : idx_right + 1]
        interp_vio_eul = self.vio_eul[idx_left : idx_right + 1, :]
        vio_euls_uw = unwrap_rpy(interp_vio_eul)
        vio_eul_uw = interp1d(interp_vio_ts, vio_euls_uw, axis=0)(ts_oldest_state)
        vio_eul = np.deg2rad(wrap_rpy(vio_eul_uw))
        ts_interp = np.array([ts_oldest_state, ts_end])
        vio_interp = interp1d(self.vio_ts, self.vio_p, axis=0)(ts_interp)
        vio_meas = vio_interp[1] - vio_interp[0]  # simulated displacement measurement
        meas_cov = np.diag(np.array([1e-3, 1e-3, 1e-3]))
        # express in gravity aligned frame bty normalizing on yaw
        Ri_z = Rotation.from_euler("z", vio_eul[2]).as_matrix()
        meas = Ri_z.T.dot(vio_meas.reshape((3, 1)))
        return meas, meas_cov, True
        
