'''
Inputs:
- csv_fn: .csv containing trajectory

Output:
- .txt containing time, position, and orientation
'''

import argparse
import os

import csv
import numpy as np


if __name__ == '__main__':
	parser = argparse.ArgumentParser()
	parser.add_argument("--in_dir", type=str)
	parser.add_argument("--n_seq", type=int)
	args = parser.parse_args()

	in_dir = args.in_dir
	n_seq = args.n_seq

	for i in range(n_seq):
		csv_fn = os.path.join(in_dir, 'csv/%s.csv' % str(i+1))
		# read data
		with open(csv_fn, 'r') as csvfile:
			traj_reader = csv.reader(csvfile, delimiter=',', quotechar='|')
			traj_array = np.array(list(traj_reader))[1:]
			traj = traj_array.astype(float)

		# write to txt
		out_filename = os.path.join(in_dir, 'txt/%s.txt' % str(i+1))
		f = open(out_filename, 'w')
		f.write('# timestamp tx ty tz qx qy qz qw\n')
		for t in traj:
			f.write('%.12f %.12f %.12f %.12f %.12f %.12f %.12f %.12f\n' % 
				(t[0], t[1], t[2], t[3], t[5], t[6], t[7], t[4]))

		if (i % 100) == 0:
			print('Processed %d-th .csv file.' % (i+1))

