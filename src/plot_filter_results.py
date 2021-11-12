"""
TLIO Stochastic Cloning Extended Kalman Filter
Input: IMU data
Measurement: window displacement estimates from networks
Filter states: position, velocity, rotation, IMU biases
"""
import argparse
import datetime
import IPython
import os
import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial.transform import Rotation
import scripts.tlio.plots as custom_plots
from utils.argparse_utils import add_bool_arg


def get_datalist(list_path):
    with open(list_path) as f:
        data_list = [s.strip() for s in f.readlines() if len(s.strip()) > 0]
    return data_list


def run_evaluation(args):
    # Load data
    data_list = get_datalist(args.data_list)
    for data_fn in data_list:
        data_dir = os.path.join(args.data_dir, data_fn)
        filter_dir = os.path.join(args.filter_dir, data_fn)
        
        # Load data
        gt_states = np.loadtxt(os.path.join(data_dir, "evolving_state.txt"))
        gt_ts = gt_states[:, 0] * 1e-6 # sec
        gt_p = gt_states[:, 5:8]
        gt_v = gt_states[:, 8:11]
        gt_rq = gt_states[:, 1:5]
        vio_r = Rotation.from_quat(
            np.concatenate([gt_rq[:, 1:4], np.expand_dims(gt_rq[:, 0], axis=1)], axis=1) )
        
        biases_path = os.path.join(data_dir, "Biases.txt")
        values = []
        fhand = open(biases_path)
        counter_line = -1
        for line in fhand:
            counter_line = counter_line + 1 
            if counter_line == 0: continue
            line = line.rstrip()
            contents = line.split(':')
            values.append(float(contents[1]))
        gt_acc_bias = np.array([values[0], values[1], values[2]]).reshape((3, 1))
        gt_gyro_bias = np.array([values[3], values[4], values[5]]).reshape((3, 1))

        print("Ground-truth acc. bias")
        print(gt_acc_bias)
        print("Ground-truth gyro. bias")
        print(gt_gyro_bias)

        filter_states = np.loadtxt(os.path.join(filter_dir, "not_vio_state.txt"), delimiter=",")
        R_init = filter_states[0, :9].reshape(-1, 3, 3)
        r_init = Rotation.from_matrix(R_init)
        Rs = filter_states[:, :9].reshape(-1, 3, 3)
        rs = Rotation.from_matrix(Rs)
        euls = rs.as_euler("xyz", degrees=True)
        vs = filter_states[:, 9:12]
        filter_p = filter_states[:, 12:15]
        ps_dr = np.cumsum(
            filter_states[:, 9:12] * np.diff(filter_states[:, 27:28], prepend=filter_states[0, 27], axis=0),
            axis=0,
        )
        ba = filter_states[:, 15:18]
        bg = filter_states[:, 18:21]
        accs = filter_states[:, 21:24]  # offline calib compensated, scale+bias
        gyrs = filter_states[:, 24:27]  # offline calib compensated, scale+bias
        ts = filter_states[:, 27]

        sigma_r = np.sqrt(filter_states[:, 28:31]) * 180.0 / np.pi
        sigma_v = np.sqrt(filter_states[:, 31:34])
        sigma_p = np.sqrt(filter_states[:, 34:37])
        sigma_bg = np.sqrt(filter_states[:, 37:40])
        sigma_ba = np.sqrt(filter_states[:, 40:43])
        innos = filter_states[:, 43:46]
        meas = filter_states[:, 46:49]
        pred = filter_states[:, 49:52]
        meas_sigma = filter_states[:, 52:55]
        inno_sigma = filter_states[:, 55:58]
        nobs_sigma = filter_states[:, 58 : 58 + 16]

        # compute errors
        # rotation
        matched_ts = []
        err_ang = []
        err_R_xyz = []
        for idx_f, t in enumerate(ts):
            if round(t, 6) in gt_ts:
                idx_gt = np.where(gt_ts == round(t, 6))[0][0]

                gt_r = vio_r[idx_gt]
                filter_r = rs[idx_f] 
                dR = gt_r * filter_r.inv()
                err_xyz = dR.as_euler("xyz", degrees=True)

                rot_angle = np.arccos((np.trace(dR.as_matrix()) - 1 - 1e-6) / 2)

                matched_ts.append(t)
                err_ang.append(np.rad2deg(rot_angle))
                err_R_xyz.append(err_xyz)

        err_ang = np.asarray(err_ang)
        err_R_xyz = np.asarray(err_R_xyz)

        # plots
        fig1 = plt.figure(num="Compared Filter vs Groundtruth")
        custom_plots.plotFilterVsGtPosXY(filter_p, gt_p)

        fig2 = plt.figure(num="Predicted acc. bias")
        custom_plots.plotImuBiases(ts, ba)

        fig3 = plt.figure(num="Predicted gyro. bias")
        custom_plots.plotImuBiases(ts, bg)

        # err rot angle norm
        fig4 = plt.figure(num="Error norm of rot. angle")
        custom_plots.plotArray(matched_ts, err_ang, 'Error norm of rot. angle', \
            ['ts', 'ang. norm'])

        # err rot euler angles
        fig5 = plt.figure(num="Error euler angles")
        custom_plots.plotRotErrEulerXYZ(matched_ts, err_R_xyz)

        outdir = os.path.join(args.filter_dir, data_fn)

        # save plots
        if args.save_plot:
            fig1.savefig(os.path.join(outdir, "filter_vs_gt.png"))
            fig2.savefig(os.path.join(outdir, "acc_bias.png"))
            fig3.savefig(os.path.join(outdir, "gyro_bias.png"))
            fig4.savefig(os.path.join(outdir, "err_norm_rot_angle.png"))
            fig5.savefig(os.path.join(outdir, "err_norm_euler_angles.png"))
            fig1.clear()
            fig2.clear()
            fig3.clear()
            fig4.clear()
            fig5.clear()
        else:
            plt.show()

        # save trajectory
        filter_freq = 1.0 / (ts[1]-ts[0])
        sampling_freq = 30.0
        sampling_step = int(filter_freq/ sampling_freq)
        outfn = os.path.join(outdir, 'stamped_traj_estimate.txt')
        stamped_traj = []
        for i in range(0, ts.shape[0], sampling_step):
            stamped_traj.append(
                np.array([
                    ts[i], 
                    filter_p[i,0], filter_p[i,1], filter_p[i,2],
                    rs[i].as_quat()[0], rs[i].as_quat()[1], rs[i].as_quat()[2], rs[i].as_quat()[3]
                ])
            )

        np.savetxt(outfn, np.asarray(stamped_traj), header='ts x y z qx qy qz qw', fmt='%.6f')

        outfn = os.path.join(outdir, 'stamped_groundtruth.txt')
        stamped_traj = []
        for i in range(0, gt_ts.shape[0], sampling_step):
            stamped_traj.append(
                np.array([
                    gt_ts[i], 
                    gt_p[i,0], gt_p[i,1], gt_p[i,2],
                    gt_rq[i,1], gt_rq[i,2], gt_rq[i,3], gt_rq[i,0],
                ])
            )
        np.savetxt(outfn, np.asarray(stamped_traj), header='ts x y z qx qy qz qw', fmt='%.6f')
        
if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--data_dir", type=str)
    parser.add_argument("--data_list", type=str, default="test.txt")
    parser.add_argument("--filter_dir", type=str)
    add_bool_arg(parser, "save_plot", default=True)
    args = parser.parse_args()
    print(vars(args))
    run_evaluation(args)
