'''
This script writes poses from evolving_state.txt (see: https://github.com/gcioffi/TLIO/tree/master) to a .csv file that can be used 
for handeye calibration (https://github.com/ethz-asl/hand_eye_calibration/tree/master)

Input:
- ev_state_fn: path to evolving_state.txt

Output:
- .csv file ready to be used for handeye calibration
'''

import argparse
import os

import numpy as np


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("--ev_state_fn", type=str)
    args = parser.parse_args()

    states = np.loadtxt(args.ev_state_fn)
    t0 = states[0,0]

    csvfn = os.path.join(os.path.dirname(args.ev_state_fn), 
    	'poses_' + os.path.basename(args.ev_state_fn)[:-4] + '.csv')
    csv_file = open(csvfn, 'w')
    for s in states:
    	csv_file.write(str(s[0] - t0) + ', ' + 
    		str(s[5]) + ', ' + 
    		str(s[6]) + ', ' + 
    		str(s[7]) + ', ' + 
    		str(s[2]) + ', ' + 
    		str(s[3]) + ', ' + 
    		str(s[4]) + ', ' + 
    		str(s[1]) + '\n')

    csv_file.close()

