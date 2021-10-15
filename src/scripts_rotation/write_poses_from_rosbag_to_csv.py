'''
This scripts reads PoseStamped topics from a rosbag and writes them to a .csv file (format: t x y z qx qy qz qw)

Inputs:
- bagfn: Path to the rosbag
- topic: Name of the ROS topic containing the poses

Ouput:
- .csv file (format: t x y z qx qy qz qw)
'''

import argparse
import os

import numpy as np
import rosbag


def run(bagfn, topicfn):
	csvfn = os.path.join(os.path.dirname(bagfn), os.path.basename(bagfn)[:-4] + '.csv')

	csv_file = open(csvfn, 'w')
	with rosbag.Bag(bagfn, 'r') as bag:
		for (topic, msg, ts) in bag.read_messages():
			if topic == topicfn:
				csv_file.write(str(msg.header.stamp.to_sec()) + ', ' + 
					str(msg.pose.position.x) + ', ' + 
					str(msg.pose.position.y) + ', ' + 
					str(msg.pose.position.z) + ', ' + 
					str(msg.pose.orientation.x) + ', ' + 
					str(msg.pose.orientation.y) + ', ' + 
					str(msg.pose.orientation.z) + ', ' + 
					str(msg.pose.orientation.w) + '\n')
	csv_file.close()


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("--bagfn", type=str)
    parser.add_argument("--topic", type=str)
    args = parser.parse_args()

    run(args.bagfn, args.topic)

