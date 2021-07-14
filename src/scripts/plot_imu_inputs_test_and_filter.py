'''
This script plots a comparison between imu inputs as seen by the network during test and filter mode.
'''

import argparse
import matplotlib.pyplot as plt
import numpy as np
import os


def plotData(args):
    # Load data
    net_in_gyro_x_fn = os.path.join(args.network_dir, 'net_in_gyro_x.txt')
    net_in_gyro_y_fn = os.path.join(args.network_dir, 'net_in_gyro_y.txt')
    net_in_gyro_z_fn = os.path.join(args.network_dir, 'net_in_gyro_z.txt')

    net_in_acc_x_fn = os.path.join(args.network_dir, 'net_in_acc_x.txt')
    net_in_acc_y_fn = os.path.join(args.network_dir, 'net_in_acc_y.txt')
    net_in_acc_z_fn = os.path.join(args.network_dir, 'net_in_acc_z.txt')

    filter_in_gyro_x_fn = os.path.join(args.filter_dir, 'net_in_gyro_x.txt')
    filter_in_gyro_y_fn = os.path.join(args.filter_dir, 'net_in_gyro_y.txt')
    filter_in_gyro_z_fn = os.path.join(args.filter_dir, 'net_in_gyro_z.txt')

    filter_in_acc_x_fn = os.path.join(args.filter_dir, 'net_in_acc_x.txt')
    filter_in_acc_y_fn = os.path.join(args.filter_dir, 'net_in_acc_y.txt')
    filter_in_acc_z_fn = os.path.join(args.filter_dir, 'net_in_acc_z.txt')

    net_in_gyro_x = np.loadtxt(net_in_gyro_x_fn)
    net_in_gyro_y = np.loadtxt(net_in_gyro_y_fn)
    net_in_gyro_z = np.loadtxt(net_in_gyro_z_fn)

    net_in_acc_x = np.loadtxt(net_in_acc_x_fn)
    net_in_acc_y = np.loadtxt(net_in_acc_y_fn)
    net_in_acc_z = np.loadtxt(net_in_acc_z_fn)

    filter_in_gyro_x = np.loadtxt(filter_in_gyro_x_fn)[1:, 1:]
    filter_in_gyro_y = np.loadtxt(filter_in_gyro_y_fn)[1:, 1:]
    filter_in_gyro_z = np.loadtxt(filter_in_gyro_z_fn)[1:, 1:]

    filter_in_acc_x = np.loadtxt(filter_in_acc_x_fn)[1:, 1:]
    filter_in_acc_y = np.loadtxt(filter_in_acc_y_fn)[1:, 1:]
    filter_in_acc_z = np.loadtxt(filter_in_acc_z_fn)[1:, 1:]

    print("loaded %d net. inputs, %d filter inputs" % \
        (net_in_gyro_x.shape[0], filter_in_gyro_x.shape[0]))

    # Plotting
    # gyro
    fig0, axs0 = plt.subplots(3)
    fig0.suptitle('Network vs filter gyro inputs')

    n = net_in_gyro_x.shape[1]
    m = filter_in_gyro_x.shape[1]
    
    axs0[0].plot(net_in_gyro_x.reshape(-1), label='net gyro x')
    '''for i, w in enumerate(net_in_gyro_x):
        axs0[0].plot(int((i+1)*n-1), w[-1], 'x')'''

    axs0[0].plot(filter_in_gyro_x.reshape(-1), label='filter gyro x')
    
    axs0[0].grid()
    axs0[0].legend()

    axs0[1].plot(net_in_gyro_y.reshape(-1), label='net gyro y')

    axs0[1].plot(filter_in_gyro_y.reshape(-1), label='filter gyro y')

    axs0[1].grid()
    axs0[1].legend()

    axs0[2].plot(net_in_gyro_z.reshape(-1), label='net gyro z')

    axs0[2].plot(filter_in_gyro_z.reshape(-1), label='filter gyro z')
    
    axs0[2].grid()
    axs0[2].legend()

    # acc
    fig1, axs1 = plt.subplots(3)
    fig1.suptitle('Network vs filter acc inputs')
    
    axs1[0].plot(net_in_acc_x.reshape(-1), label='net acc x')
    axs1[0].plot(filter_in_acc_x.reshape(-1), label='filter acc x')
    
    axs1[0].grid()
    axs1[0].legend()

    axs1[1].plot(net_in_acc_y.reshape(-1), label='net acc y')
    axs1[1].plot(filter_in_acc_y.reshape(-1), label='filter acc y')

    axs1[1].grid()
    axs1[1].legend()

    axs1[2].plot(net_in_acc_z.reshape(-1), label='net acc z')
    axs1[2].plot(filter_in_acc_z.reshape(-1), label='filter acc z')
    
    axs1[2].grid()
    axs1[2].legend()

    plt.show()


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("--network_dir", type=str, default='/home/giovanni/TLIO/results/race_track_18Jun21/results/network/seq17')
    parser.add_argument("--filter_dir", type=str, default='/home/giovanni/TLIO/results/race_track_18Jun21/results/filter/seq17')
    args = parser.parse_args()
    plotData(args)

