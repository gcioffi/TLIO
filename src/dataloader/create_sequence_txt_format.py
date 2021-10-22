import argparse
import os

import numpy as np
from scipy import interpolate
import yaml

SMALL_EPS = 0.0001
BIG_EPS = 0.002


# @ToDo: move to a different script
def loadConfig(config_yaml):
    config = {}
    config['traj_fn'] = config_yaml['traj_fn']
    config['vel_fn'] = config_yaml['vel_fn']
    config['sequence_dir'] = config_yaml['sequence_dir']
    config['imu_fn'] = config_yaml['imu_fn']
    config['gt_freq'] = config_yaml['gt_freq']
    config['imu_freq'] = config_yaml['imu_freq']
    config['interpolate_imu'] = config_yaml['interpolate_imu']
    config['desired_imu_freq'] = config_yaml['desired_imu_freq']
    return config


def createDataset(config):
    sequence_dir = config['sequence_dir']

    # load trajectory
    traj_fn = config['traj_fn']
    traj = np.loadtxt(traj_fn)
    # load velocity
    vel_fn = config['vel_fn']
    vel = np.loadtxt(vel_fn)
    # load IMU meas
    imu_fn = config['imu_fn']
    imu_freq = config['imu_freq']
    imu_fn = os.path.join(sequence_dir, imu_fn)
    imu_meas = np.loadtxt(imu_fn)

    # find first traj meas newer than first imu meas
    idx_first_traj_meas = 0
    while traj[idx_first_traj_meas, 0] < imu_meas[0,0] - SMALL_EPS:
        idx_first_traj_meas += 1
    # find first traj meas newer than last imu meas
    idx_last_traj_meas = idx_first_traj_meas
    while traj[idx_last_traj_meas, 0] < imu_meas[-1,0] - SMALL_EPS:
        idx_last_traj_meas += 1
        if idx_last_traj_meas == traj.shape[0] - 1:
            break
    # sample traj
    traj = traj[idx_first_traj_meas:idx_last_traj_meas+1, :]
    # sample vel
    vel = np.array([v for v in vel if traj[0,0] - SMALL_EPS <= v[0] <= traj[-1,0] + SMALL_EPS])

    # This code is only relevant if we use the vicon traj (= traj_fn) instead that the spline.
    # use start and end time of vels to get the relevant part of the vicon trajectory
    # start and end time of the velocity are the first and last possible times to sample from the bspline
    if np.abs(imu_meas[0,0] - traj[0,0]) > SMALL_EPS:
        print('[WARNING] GT has not been sampled from the spline. Make sure this is really what you want !!')    
        t_start = vel[0,0]
        t_end = vel[-1,0]
        traj = np.array([t for t in traj if t_start <= t[0] <= t_end])

        # check that ts match in traj and vel
        # if not remove additional velocities. 
        # This is due to the fact that same samples might be missing in the vicon traj.
        if vel.shape[0] > traj.shape[0]:
            tmp = []
            cnt = 0
            for t in traj:
                while np.abs(t[0]-vel[cnt,0]) > BIG_EPS:
                    cnt += 1
                    if cnt == vel.shape[0]:
                        break
                if cnt == vel.shape[0]:
                        break
                tmp.append(vel[cnt])
            vel = np.asarray(tmp)

        assert vel.shape[0] == traj.shape[0], \
        'Number of velocity (=%d) and trajectory (=%d) measurements do not match!' % (vel.shape[0], traj.shape[0])
        # make sure traj and vel ts are the same
        for i, (t,v) in enumerate(zip(traj, vel)): 
            assert np.abs(t[0]-v[0]) < BIG_EPS, \
            'Time traj sample (=%.4f) different from vel sample (=%.4f)' % (t[0], v[0])
    else:
        assert vel.shape[0] == traj.shape[0], \
        'Number of velocity (=%d) and trajectory (=%d) measurements do not match!' % (vel.shape[0], traj.shape[0])
        # make sure traj and vel ts are the same
        for i, (t,v) in enumerate(zip(traj, vel)): 
            assert np.abs(t[0]-v[0]) < SMALL_EPS, \
            'Time traj sample (=%.4f) different from vel sample (=%.4f)' % (t[0], v[0])

    # interpolate
    if (config['interpolate_imu']):
        print('[WARNING] Interpolating IMU. Make sure this is really what you want !!')
        f_interp = interpolate.interp1d(imu_meas[:,0], imu_meas[:,1:].T)
        desired_imu_freq = config['desired_imu_freq']
        dt = 1.0 / config['desired_imu_freq']
        interp_t = []
        curr_t = imu_meas[0,0]
        while curr_t <= imu_meas[-1,0]:
            interp_t.append(curr_t)
            curr_t += dt
        interp_imu_meas = f_interp(np.asarray(interp_t))
        interp_imu_meas = interp_imu_meas.T
        imu_meas = np.hstack((np.asarray(interp_t).reshape((-1,1)), interp_imu_meas))
        imu_freq = desired_imu_freq

    if np.abs(imu_meas[0,0] - traj[0,0]) > SMALL_EPS:
        # find imu meas with ts corresponding to a traj meas
        # this is the has_gt vector.
        has_gt = np.zeros((imu_meas.shape[0],), dtype=int)
        done = False
        cnt = 0
        for i in range(0, traj.shape[0]):
            t = traj[i]
            while np.abs(t[0] - imu_meas[cnt, 0]) > BIG_EPS:
                cnt += 1
                if cnt == imu_meas.shape[0]:
                    done = True
                    break
            if done:
                break
            has_gt[cnt] = 1

        # check that has_gt has "enough" elements
        gt_freq = config['gt_freq']
        r = imu_freq / gt_freq
        n = imu_meas.shape[0] / r
        assert np.sum(has_gt) > (n - 0.1*n), 'has_gt does not have enough elements (%d) / (%d)' \
        % (np.sum(has_gt), imu_meas.shape[0])
    else:
        has_gt = np.ones((imu_meas.shape[0],), dtype=int)
        # we do not sample imu from the last 20 ms of the spline.
        # the reason is that it might happen that those imu meas have 'unrealistic' peaks
        # we do not sample the last 30 imu meas after butterworth filtering
        # the reason is that it might happen that those imu meas have 'unrealistic' peaks
        assert imu_meas.shape[0] == traj.shape[0], \
        'len imu (=%d) != len traj (=%d)' % (imu_meas.shape[0], traj.shape[0])
        for ti, tj in zip(imu_meas[:,0], traj[:,0]):
            assert np.abs(ti-tj) < SMALL_EPS, \
            't imu (=%.f) != t traj (=%.f)' % (ti, tj)

    # Saving
    # my_timestamps_p
    out_fn = os.path.join(sequence_dir, 'my_timestamps_p.txt')
    # TLIO expects times in microsec. here
    traj_ts_ms = traj[:, 0] * 1e6
    np.savetxt(out_fn, traj_ts_ms)

    # imu_measurements
    out_fn = os.path.join(sequence_dir, 'imu_measurements.txt')
    # Format imu_measurements.txt = [t (in microsec), acc_raw (3), acc_cal (3), gyr_raw (3), gyr_cal (3), has_gt]
    # TLIO expects times in microsec. here
    ti = imu_meas[:, 0] * 1e6
    # raw = calib since we simulate 0 mean bias
    acc_raw = imu_meas[:, 4:]
    acc_cal = imu_meas[:, 4:]
    gyro_raw = imu_meas[:, 1:4]
    gyro_cal = imu_meas[:, 1:4]
    imu_meas_tlio_format = np.hstack((ti.reshape((-1,1)), 
        acc_raw, acc_cal, gyro_raw, gyro_cal, has_gt.reshape((-1,1))))
    np.savetxt(out_fn, imu_meas_tlio_format)

    # Get evolving state = [t (in microsec), q_wxyz (4), p (3), v (3)]
    evolving_state = []
    for (t,v) in zip(traj, vel):
        state = np.array([t[0] * 1e6, 
            t[7], t[4], t[5], t[6], 
            t[1], t[2], t[3], 
            v[1], v[2], v[3]])
        evolving_state.append(state)
    evolving_state = np.asarray(evolving_state)
    out_fn = os.path.join(sequence_dir, 'evolving_state.txt')
    np.savetxt(out_fn, evolving_state)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("--config_fn", type=str)
    args = parser.parse_args()
    
    f = open(args.config_fn)
    config = loadConfig(yaml.load(f, Loader=yaml.FullLoader))
    f.close()

    createDataset(config)

