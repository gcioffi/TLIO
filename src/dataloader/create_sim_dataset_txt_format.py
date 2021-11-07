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
    config['first_seq_id'] = config_yaml['first_seq_id']
    return config


def createDataset(config):
    dataset_dir = config['dataset_dir']
    n_seq = config['n_seq']
    first_seq_id = config['first_seq_id']

    for i in range(first_seq_id, first_seq_id+n_seq, 1):
        seq_dir = os.path.join(dataset_dir, 'seq' + str(i))

        # load
        traj_fn = os.path.join(seq_dir, config['traj_fn']) 
        traj = np.loadtxt(traj_fn)
        # load velocity
        vel_fn = os.path.join(seq_dir, config['vel_fn'])
        vel = np.loadtxt(vel_fn)

        if config['use_filtered_imu']:
            imu_fn = os.path.join(seq_dir, 'filtered_simulated_imu_meas.txt')
        else:
            imu_fn = os.path.join(seq_dir, 'simulated_imu_meas.txt')
        imu_ts = np.loadtxt(imu_fn)[:,0]
        imu_meas = np.loadtxt(imu_fn)

        if config['use_filtered_imu']:
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
            vel = np.array([v for v in vel if (traj[0,0] - SMALL_EPS) <= v[0] <= (traj[-1,0] + SMALL_EPS)])

        assert imu_meas.shape[0] == traj.shape[0], \
            'len imu (=%d) != len traj (=%d)' % (imu_meas.shape[0], traj.shape[0])
        for ti, tj in zip(imu_meas[:,0], traj[:,0]):
            assert np.abs(ti-tj) < SMALL_EPS, \
            't imu (=%.f) != t traj (=%.f)' % (ti, tj)

        has_gt = np.ones((imu_meas.shape[0],), dtype=int)

        assert vel.shape[0] == traj.shape[0], \
        'Number of velocity (=%d) and trajectory (=%d) measurements do not match!' % (vel.shape[0], traj.shape[0])
        # make sure traj and vel ts are the same
        for _, (t,v) in enumerate(zip(traj, vel)): 
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

