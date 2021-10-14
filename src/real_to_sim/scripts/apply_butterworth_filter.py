'''
This script applies a butterworth filter to imu measurements

Inputs:
- imu_fn.txt : Imu measurements. Format [time_sec gyr_x gyr_y gyr_z  acc_x acc_y acc_z]
- freq: frequency of the measurements.

Outputs:
- .txt containing the filtered measurements.

'''


import argparse
import os

import numpy as np
from scipy import signal
import yaml


def loadConfig(config_yaml):
    config = {}
    config['n_seq'] = config_yaml['n_seq']
    config['imu_dir'] = config_yaml['imu_dir']
    return config


class ButterworthFilter(object):
    def __init__(self, freq = 1000, order = 4, cutoff_freq_hz = 50, ftype = 'lowpass'):
        self.sos = signal.butter(order, cutoff_freq_hz, ftype, fs=freq, output='sos')

    def apply(self, x):
        x_filtered = signal.sosfilt(self.sos, x)
        return x_filtered


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("--signal_fn", type=str)
    parser.add_argument("--freq", type=float, help='freq of the signal in Hz')
    parser.add_argument("--config", type=str)
    args = parser.parse_args()
    
    if args.config == '':
        freq = args.freq

        butterworthFilter = ButterworthFilter(freq)

        # read measurements
        x_with_time = np.loadtxt(args.signal_fn)

        # low pass
        ts = x_with_time[:, 0]
        x = x_with_time[:, 1:]

        x_filtered = butterworthFilter.apply(x.T)
        x_filtered = np.hstack((ts.reshape(-1,1), x_filtered.T))

        # remove first 30 ms of meas since they might be affected by the filtering
        x_filtered = x_filtered[30:, :]
    
        outfn = os.path.join(os.path.dirname(args.signal_fn), 'filtered_' + os.path.basename(args.signal_fn))
        print('Saving filtered measurements to %s' % outfn)
        np.savetxt(outfn, x_filtered, header='timestamp wx wy wz ax ay az')

    else:
        f = open(args.config)
        config = loadConfig(yaml.load(f, Loader=yaml.FullLoader))
        f.close()

        n_seq = config['n_seq']
        imu_dir = config['imu_dir']

        butterworthFilter = ButterworthFilter()

        for i in range(n_seq):
            # read measurements
            signal_fn = os.path.join(imu_dir, 'seq' + str(i+1) + '/simulated_imu_meas.txt')
            x_with_time = np.loadtxt(signal_fn)

            # low pass
            ts = x_with_time[:, 0]
            x = x_with_time[:, 1:]

            x_filtered = butterworthFilter.apply(x.T)
            x_filtered = np.hstack((ts.reshape(-1,1), x_filtered.T))

            # remove first 30 ms of meas since they might be affected by the filtering
            x_filtered = x_filtered[30:, :]
        
            outfn = os.path.join(os.path.dirname(signal_fn), 'filtered_' + os.path.basename(signal_fn))
            #print('Saving filtered measurements to %s' % outfn)
            np.savetxt(outfn, x_filtered, header='timestamp wx wy wz ax ay az')

            if i % 10 == 0:
                print('Processing sequence %d' % i)

