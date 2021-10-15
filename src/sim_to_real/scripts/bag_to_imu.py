#!/usr/bin/python

import rosbag
import argparse

def extract(bagfile, imu_topic, t_start, t_end, out_filename):
    n = 0
    f = open(out_filename, 'w')
    f.write('# timestamp ang_vel_x ang_vel_y ang_vel_z lin_acc_x lin_acc_y lin_acc_z\n')    
    with rosbag.Bag(bagfile, 'r') as bag:
        for (topic, msg, ts) in bag.read_messages(topics=str(imu_topic)):
            t = msg.header.stamp.to_sec()
            if (t >= t_start) and (t <= t_end):
                f.write('%.12f %.12f %.12f %.12f %.12f %.12f %.12f\n' % 
                        (msg.header.stamp.to_sec(),
                         msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z,
                         msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z))
                n += 1
    print('wrote ' + str(n) + ' imu messages to the file: ' + out_filename)
          
if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='''
    Extracts IMU messages from bagfile.
    ''')
    parser.add_argument('bag', help='Bagfile')
    parser.add_argument('topic', help='Topic')
    parser.add_argument('t_start', default=-1.0, type=float, help='Initial time')
    parser.add_argument('t_end', default=1e19, type=float, help='Final time')
    args = parser.parse_args()
    
    extract(args.bag, args.topic, args.t_start, args.t_end, 'imu.txt')
