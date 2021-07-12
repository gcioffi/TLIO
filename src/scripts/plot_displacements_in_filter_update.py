import argparse
import matplotlib.pyplot as plt
import numpy as np
import os


def plotData(data_dir):
    # Load data
    net_d_fn = os.path.join(data_dir, 'net_disp.txt')
    vio_d_fn = os.path.join(data_dir, 'vio_disp.txt')

    net_d = np.loadtxt(net_d_fn)
    vio_d = np.loadtxt(vio_d_fn)

    x_net = np.arange(0, net_d.shape[0])
    x_vio = np.arange(0, vio_d.shape[0])

    n = np.minimum(len(x_net), len(x_vio))
    err_d = net_d[0:n, :] - vio_d[0:n, :]

    fig0, axs = plt.subplots(3)
    fig0.suptitle('Network vs VIO displacements')

    axs[0].plot(x_net, net_d[:,0], label='d net x')
    axs[0].plot(x_vio, vio_d[:,0], label='d vio x')

    axs[0].grid()
    axs[0].legend()

    axs[1].plot(x_net, net_d[:,1], label='d net y')
    axs[1].plot(x_vio, vio_d[:,1], label='d vio y')

    axs[1].grid()
    axs[1].legend()

    axs[2].plot(x_net, net_d[:,2], label='d net z')
    axs[2].plot(x_vio, vio_d[:,2], label='d vio z')

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
    parser.add_argument("--data_dir", type=str, default='/home/giovanni/TLIO/results/race_track_18Jun21/results/filter/seq17')
    args = parser.parse_args()
    data_dir = args.data_dir
    plotData(data_dir)

