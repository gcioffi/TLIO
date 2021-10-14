import argparse
import os

import numpy as np
import yaml

# @ToDo: move to a different script
def loadConfig(config_yaml):
    config = {}
    config['n_seq'] = config_yaml['n_seq']
    config['traj_fn'] = config_yaml['traj_fn']
    config['dataset_dir'] = config_yaml['dataset_dir']
    config['t_start'] = config_yaml['start_t']
    config['t_end'] = config_yaml['end_t']
    config['gt_freq'] = config_yaml['gt_freq']
    return config


def createDataset(config):
    dataset_dir = config['dataset_dir']
    n_seq = config['n_seq']

    # load trajectory
    traj_fn = config['traj_fn']
    vicon_traj = np.loadtxt(traj_fn)
    # load velocity
    vel_fn = os.path.join(dataset_dir, 'velocity.txt')
    vel = np.loadtxt(vel_fn)

    # use start and end time of vels to get the relevant part of the vicon trajectory
    # start and end time of the velocity are the first and last possible times to sample from the bspline
    t_start = vel[0,0]
    t_end = vel[-1,0]
    traj = np.array([t for t in vicon_traj if t_start <= t[0] <= t_end])

    # check that ts match in traj and vel
    # if not remove additional velocities. 
    # This is due to the fact that same samples might be missing in the vicon traj.
    if vel.shape[0] > traj.shape[0]:
        tmp = []
        cnt = 0
        for t in traj:
            while np.abs(t[0]-vel[cnt,0]) > 0.002:
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
        assert np.abs(t[0]-v[0]) < 0.002, \
        'Time traj sample (=%.4f) different from vel sample (=%.4f)' % (t[0], v[0])


    # debug
    '''print(vel.shape)
                import matplotlib.pyplot as plt
                plt.figure(0)
                plt.plot(vel[:,1], label='x')
                plt.plot(vel[:,2], label='y')
                plt.plot(vel[:,3], label='z')
                plt.legend()
                plt.show()
                assert False'''
    # end

    # Get evolving state = [t (in microsec), q_wxyz (4), p (3), v (3)]
    evolving_state = []
    for (t,v) in zip(traj, vel):
        state = np.array([t[0] * 1e6, 
            t[7], t[4], t[5], t[6], 
            t[1], t[2], t[3], 
            v[1], v[2], v[3]])
        evolving_state.append(state)
    evolving_state = np.asarray(evolving_state)

    # debug
    #n_seq = 5
    # end
    for i in range(n_seq):
        seq_dir = os.path.join(dataset_dir, 'seq' + str(i+1))
        # load IMU meas
        imu_fn = os.path.join(seq_dir, 'simulated_imu_meas.txt')
        imu_meas = np.loadtxt(imu_fn)

        # find imu meas with ts corresponding to a traj meas
        # this is the has_gt vector.
        has_gt = np.zeros((imu_meas.shape[0],), dtype=int)
        done = False
        cnt = 0
        for t in traj:
            while np.abs(t[0] - imu_meas[cnt, 0]) > 0.0005:
                cnt += 1
                if cnt == imu_meas.shape[0]:
                    done = True
                    break
            if done:
                break

            has_gt[cnt] = 1

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


        # debug
        '''if has_gt[cnt] == 1:
                                                print('t[0]= %.12f, imu_meas[cnt,0]= %.12f' % (t[0], imu_meas[cnt, 0]))'''
        # end

        # debug
        #print('Has vio contains %d elements' % np.sum(has_gt))
        # end

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

