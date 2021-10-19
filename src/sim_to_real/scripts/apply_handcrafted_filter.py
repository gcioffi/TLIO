'''
This script applies a custom made filter to real imu measurements on z axis in order to remove noise due to undumped imu

Inputs:
- signal_fn.txt : Imu measurements. Format [time_sec gyr_x gyr_y gyr_z  acc_x acc_y acc_z]
- freq: frequency of the measurements.

Outputs:
- .txt containing the filtered measurements.

'''


import argparse
import os

import matplotlib.pyplot as plt
import numpy as np
import yaml


def loadConfig(config_yaml):
    config = {}
    config['n_seq'] = config_yaml['n_seq']
    config['imu_dir'] = config_yaml['imu_dir']
    return config


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("--signal_fn", type=str)
    parser.add_argument("--freq", type=float, help='freq of the signal in Hz')
    parser.add_argument("--window_len_sec", type=float, help='len. of the moving avg window in sec')
    parser.add_argument("--noise_std", type=float, help='std of gaussian noise')
    args = parser.parse_args()
    
    freq = args.freq
    freq_sqrt = np.sqrt(args.freq)
    window_len_sec = args.window_len_sec
    noise_std = args.noise_std

    # read measurements
    imu_meas = np.loadtxt(args.signal_fn)
    # we only work on az
    x = imu_meas[:, 6]
    N = x.shape[0]

    # compute moving average
    n_samples = int(freq * window_len_sec)
    n_samples_half = int(n_samples / 2)

    mean_signal = np.zeros((N,))
    mean_signal[:n_samples_half] = np.ones((n_samples_half,)) * np.mean(x[:n_samples_half])
    mean_signal[N-n_samples_half:] = np.ones((n_samples_half,)) * np.mean(x[N-n_samples_half:])

    std_signal = np.zeros((N,))
    std_signal[:n_samples_half] = np.ones((n_samples_half,)) * np.std(x[:n_samples_half])
    std_signal[N-n_samples_half:] = np.ones((n_samples_half,)) * np.std(x[N-n_samples_half:])
    
    for i in range(n_samples_half, (N - n_samples_half)):
        mean_signal[i] = np.mean(x[(i-n_samples_half):(i+n_samples_half)])
        std_signal[i] = np.std(x[(i-n_samples_half):(i+n_samples_half)])

    # only use past measurements
    '''mean_signal = np.zeros((N,))
    mean_signal[:n_samples] = np.ones((n_samples,)) * np.mean(x[:n_samples])
    for i in range(n_samples, N):
        mean_signal[i] = np.mean(x[(i-n_samples):i])'''

    # filter
    # for each measurements check if it is in the range mean_signal[i]+-3noise_std
    # if not resample it from a Gaussian(mean_signal[i], noise_std)
    x_filtered = np.copy(x)
    for i in range(N):
        if x[i] < mean_signal[i] - (1 * std_signal[i]) or x[i] > mean_signal[i] + (1 * std_signal[i]):
            x_filtered[i] = mean_signal[i] + np.random.normal(0.0, noise_std) * freq_sqrt

    plt.figure()
    plt.plot(x, label='original')
    plt.plot(mean_signal, label='mean avg filter')
    plt.plot(x_filtered, label='filtered', alpha=0.75)
    plt.legend()
    plt.show()

    imu_meas[:,6] = x_filtered
    # save
    outfn = os.path.join(os.path.dirname(args.signal_fn), 'filtered_' + os.path.basename(args.signal_fn))
    print('Saving filtered measurements to %s' % outfn)
    np.savetxt(outfn, imu_meas, header='timestamp wx wy wz ax ay az')

