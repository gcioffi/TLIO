#!/usr/bin/python

import os

import rosbag
import argparse

def extract(bagfile, pose_topic, topic_type, out_filename):
    n = 0
    f = open(out_filename, 'w')
    f.write('# timestamp tx ty tz qx qy qz qw\n')    
    with rosbag.Bag(bagfile, 'r') as bag:
        for (topic, msg, ts) in bag.read_messages(topics=str(pose_topic)):
            if topic_type == 'PoseWithCovarianceStamped':
                # Geometry Message PoseWithCovarianceStamped
                f.write('%.12f %.12f %.12f %.12f %.12f %.12f %.12f %.12f\n' % 
                        (msg.header.stamp.to_sec(),
                         msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z,
                         msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w))
            elif topic_type == 'PoseStamped':
                # Geometry Message PoseStamped
                f.write('%.12f %.12f %.12f %.12f %.12f %.12f %.12f %.12f\n' % 
                        (msg.header.stamp.to_sec(),
                         msg.pose.position.x, msg.pose.position.y, msg.pose.position.z,
                         msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w))
            elif topic_type == 'PointStamped':
                # Geometry Message PointStamped
                f.write('%.12f %.12f %.12f %.12f %.12f %.12f %.12f %.12f\n' % 
                        (msg.header.stamp.to_sec(),
                         msg.point.x, msg.point.y, msg.point.z,
                         0, 0, 0, 1))
            elif topic_type == 'QuadState':
                # Agiros Message QuadState
                f.write('%.12f %.12f %.12f %.12f %.12f %.12f %.12f %.12f\n' % 
                        (msg.header.stamp.to_sec(),
                         msg.pose.position.x, msg.pose.position.y, msg.pose.position.z,
                         msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w))
            else:
                assert False, "unknown message type!!!!"
            n += 1
    print('wrote ' + str(n) + ' pose messages to the file: ' + out_filename)
          

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='''
    Extracts IMU messages from bagfile.
    ''')
    parser.add_argument('bag', help='Bagfile')
    parser.add_argument('topic', help='Topic')
    parser.add_argument('type', default='PoseStamped', help='Topic type')
    args = parser.parse_args()
    out_filename = os.path.join(os.path.dirname(args.bag), os.path.basename(args.bag)[:-3] + 'txt')
    print('Extract pose from bag '+args.bag+' in topic ' + args.topic)
    print('Saving to file '+out_filename)
    extract(args.bag, args.topic, args.type, out_filename)
