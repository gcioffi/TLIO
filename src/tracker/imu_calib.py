import os.path as osp

import numpy as np


class ImuCalib:
    def __init__(self):
        self.accelScaleInv = np.eye(3)
        self.gyroScaleInv = np.eye(3)
        self.gyroGSense = np.eye(3)
        self.accelBias = np.zeros((3, 1))
        self.gyroBias = np.zeros((3, 1))

    @classmethod
    def from_attitude_file(cls, dataset, args):
        ret = cls()
        attitude_filter_path = osp.join(args.root_dir, dataset, "atttitude.txt")
        with open(attitude_filter_path, "r") as f:
            line = f.readline()
            line = f.readline()
        init_calib = np.fromstring(line, sep=",")
        ret.accelScaleInv = init_calib[1:10].reshape((3, 3))
        ret.gyroScaleInv = init_calib[10:19].reshape((3, 3))
        ret.gyroGSense = init_calib[19:28].reshape((3, 3))
        ret.accelBias = init_calib[28:31].reshape((3, 1))
        ret.gyroBias = init_calib[31:34].reshape((3, 1))
        return ret

    @classmethod
    def from_groundtruth_bias(cls, dataset, args):
        ret = cls()
        print("root_dir", args.root_dir)
        print("dataset", dataset)
        biases_path = osp.join(args.root_dir, dataset, "Biases.txt")
        values = []
        fhand = open(biases_path)
        counter_line = -1
        for line in fhand:
            counter_line = counter_line + 1 
            if counter_line == 0: continue
            line = line.rstrip()
            contents = line.split(':')
            values.append(float(contents[1]))
        ret.accelBias = np.array([values[0], values[1], values[2]]).reshape((3, 1))
        ret.gyroBias = np.array([values[3], values[4], values[5]]).reshape((3, 1))
        print("-- Using groundtruth biases --")
        print("Acc. bias")
        print(ret.accelBias)
        print("Gyro. bias")
        print(ret.gyroBias)
        return ret

    # This is a simplified version of the original code using attitude filter.
    def calibrate_raw(self, acc, gyr):
        acc_cal = acc - self.accelBias
        gyr_cal = ( gyr - self.gyroBias )
        return acc_cal, gyr_cal

    # This is a simplified version of the original code using attitude filter.
    def scale_raw(self, acc, gyr):
        acc_cal = acc
        gyr_cal = gyr
        return acc_cal, gyr_cal
