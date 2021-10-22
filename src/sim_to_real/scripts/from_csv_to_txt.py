'''
Inputs:
- csv_fn: .csv containing trajectory

Output:
- .txt containing time, position, and orientation
'''

import argparse
import os

import csv
import matplotlib.pyplot as plt
import numpy as np
from pyquaternion import Quaternion

import pose
import transformations as tf


if __name__ == '__main__':
	parser = argparse.ArgumentParser()
	parser.add_argument("--csv_fn", type=str)
	args = parser.parse_args()

	# read data
	with open(args.csv_fn, 'r') as csvfile:
		traj_reader = csv.reader(csvfile, delimiter=',', quotechar='|')
		traj_array = np.array(list(traj_reader))[1:]
		traj = traj_array.astype(float)

	# write to txt
	out_filename = os.path.join(os.path.dirname(args.csv_fn), 
		os.path.basename(args.csv_fn)[:-3] + 'txt')
	f = open(out_filename, 'w')
	f.write('# timestamp tx ty tz qx qy qz qw\n')
	for t in traj:
		f.write('%.12f %.12f %.12f %.12f %.12f %.12f %.12f %.12f\n' % 
			(t[0], t[1], t[2], t[3], t[5], t[6], t[7], t[4]))

	print('Written to the file: ' + out_filename)

