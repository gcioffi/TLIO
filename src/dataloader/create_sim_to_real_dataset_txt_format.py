import argparse
import os

import numpy as np
import yaml


SMALL_EPS = 0.0001
BIG_EPS = 0.002


# @ToDo: move to a different script
def loadConfig(config_yaml):
    config = {}
    config['n_seq'] = config_yaml['n_seq']
    config['traj_fn'] = config_yaml['traj_fn']
    config['vel_fn'] = config_yaml['vel_fn']
    config['dataset_dir'] = config_yaml['dataset_dir']
    config['gt_freq'] = config_yaml['gt_freq']
    config['imu_freq'] = config_yaml['imu_freq']
    config['use_filtered_imu'] = config_yaml['use_filtered_imu']
    return config


def createDataset(config):
    dataset_dir = config['dataset_dir']
    n_seq = config['n_seq']

    # load trajectory
    traj_fn = os.path.join(dataset_dir, config['traj_fn']) 
    traj = np.loadtxt(traj_fn)
    # load velocity
    vel_fn = os.path.join(dataset_dir, config['vel_fn'])
    vel = np.loadtxt(vel_fn)

    # load imu times (they're the same for all seqs)
    seq_dir = os.path.join(dataset_dir, 'seq1')
    if config['use_filtered_imu']:
        imu_fn = os.path.join(seq_dir, 'filtered_simulated_imu_meas.txt')
    else:
        imu_fn = os.path.join(seq_dir, 'simulated_imu_meas.txt')
    imu_ts = np.loadtxt(imu_fn)[:,0]

    # find first traj meas newer than first imu meas
    idx_first_traj_meas = 0
    while traj[idx_first_traj_meas, 0] < imu_ts[0] - SMALL_EPS:
        idx_first_traj_meas += 1
    # find first traj meas newer than last imu meas
    idx_last_traj_meas = idx_first_traj_meas
    while traj[idx_last_traj_meas, 0] < imu_ts[-1] - SMALL_EPS:
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
    if np.abs(imu_ts[0] - traj[0,0]) > SMALL_EPS:
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

    # Get evolving state = [t (in microsec), q_wxyz (4), p (3), v (3)]
    evolving_state = []
    for (t,v) in zip(traj, vel):
        state = np.array([t[0] * 1e6, 
            t[7], t[4], t[5], t[6], 
            t[1], t[2], t[3], 
            v[1], v[2], v[3]])
        evolving_state.append(state)
    evolving_state = np.asarray(evolving_state)

    imu_freq = config['imu_freq']

    for i in range(n_seq):
        seq_dir = os.path.join(dataset_dir, 'seq' + str(i+1))
        # load IMU meas
        if config['use_filtered_imu']:
            imu_fn = os.path.join(seq_dir, 'filtered_simulated_imu_meas.txt')
        else:
            imu_fn = os.path.join(seq_dir, 'simulated_imu_meas.txt')
        imu_meas = np.loadtxt(imu_fn)

        if np.abs(imu_meas[0,0] - traj[0,0]) > SMALL_EPS:
            # find imu meas with ts corresponding to a traj meas
            # this is the has_gt vector.
            has_gt = np.zeros((imu_meas.shape[0],), dtype=int)
            done = False
            cnt = 0
            for j in range(0, traj.shape[0]):
                t = traj[j]
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
        out_fn = os.path.join(seq_dir, 'my_timestamps_p.txt')
        # TLIO expects times in microsec. here
        traj_ts_ms = traj[:, 0] * 1e6
        np.savetxt(out_fn, traj_ts_ms)

        # imu_measurements
        out_fn = os.path.join(seq_dir, 'imu_measurements.txt')
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

        # evolving_state
        out_fn = os.path.join(seq_dir, 'evolving_state.txt')
        np.savetxt(out_fn, evolving_state)

        if i % 10 == 0:
            print('Processed seq: %d' % i)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("--config_fn", type=str)
    args = parser.parse_args()
    
    f = open(args.config_fn)
    config = loadConfig(yaml.load(f, Loader=yaml.FullLoader))
    f.close()

    createDataset(config)
