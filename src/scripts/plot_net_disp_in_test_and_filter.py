import argparse
import matplotlib.pyplot as plt
import numpy as np
import os


def plotData(args):
    # Load data
    net_d_fn = os.path.join(args.network_dir, 'net_outputs.txt')
    filter_d_fn = os.path.join(args.filter_dir, 'net_disp.txt')

    net_d = np.loadtxt(net_d_fn, delimiter=',')[:, 1:4]
    filter_d = np.loadtxt(filter_d_fn)

    print("loaded %d net. disp., %d filter dis." % (net_d.shape[0], filter_d.shape[0]))

    x_net = np.arange(0, net_d.shape[0])
    x_filter = np.arange(0, filter_d.shape[0])

    n = np.minimum(len(x_net), len(x_filter))
    err_d = net_d[0:n, :] - filter_d[0:n, :]

    fig0, axs = plt.subplots(3)
    fig0.suptitle('Network vs filter displacements')

    axs[0].plot(x_net, net_d[:,0], label='d net x')
    axs[0].plot(x_filter, filter_d[:,0], label='d filter x')

    axs[0].grid()
    axs[0].legend()

    axs[1].plot(x_net, net_d[:,1], label='d net y')
    axs[1].plot(x_filter, filter_d[:,1], label='d filter y')

    axs[1].grid()
    axs[1].legend()

    axs[2].plot(x_net, net_d[:,2], label='d net z')
    axs[2].plot(x_filter, filter_d[:,2], label='d filter z')

    axs[2].grid()
    axs[2].legend()

    fig1 = plt.figure(num='Errors')
    plt.plot(np.arange(0, n), err_d[:, 0], label='err x')
    plt.plot(np.arange(0, n), err_d[:, 1], label='err y')
    plt.plot(np.arange(0, n), err_d[:, 2], label='err z')

    plt.legend()
    plt.grid()

    plt.show()


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("--network_dir", type=str, default='/home/giovanni/TLIO/results/race_track_18Jun21/results/network/seq17')
    parser.add_argument("--filter_dir", type=str, default='/home/giovanni/TLIO/results/race_track_18Jun21/results/filter/seq17')
    args = parser.parse_args()
    plotData(args)

