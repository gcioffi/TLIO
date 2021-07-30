
from bagpy import bagreader
import argparse
import os
import rosbag


def checkValidity(bagfile):
    imu_present = False
    odom_present = False
    bag = rosbag.Bag(bagfile)
    print("Analyzing", bagfile)

    topics = list(bag.get_type_and_topic_info()[1].keys())
    types = []
    msg_number = []
    for i in range(0,len(list(bag.get_type_and_topic_info()[1].values()))):
        types.append(list(bag.get_type_and_topic_info()[1].values())[i][0])
        msg_number.append(list(bag.get_type_and_topic_info()[1].values())[i][1])


    if len(msg_number)!=2:
        print("One topic is missing!")
        return

    if msg_number[0] == 0 or msg_number[1] == 0:
        print("One topic has no msgs!")

    if topics[0] == '/kingfisher/ground_truth/imu' or topics[0] == 'kingfisher/ground_truth/imu':
        imu_present = True

    if topics[1] == '/kingfisher/ground_truth/imu' or topics[1] == 'kingfisher/ground_truth/imu':
        imu_present = True

    if topics[0] == '/kingfisher/ground_truth/odometry' or topics[0] == 'kingfisher/ground_truth/odometry':
        odom_present = True

    if topics[1] == '/kingfisher/ground_truth/odometry' or topics[1] == 'kingfisher/ground_truth/odometry':
        odom_present = True

    if imu_present == False:
        print("Error at bagfile", bagfile)
        print("IMU topic is missing!")

    if odom_present == False:
        print("Error at bagfile", bagfile)
        print("Odometry topic is missing!")



if __name__ == '__main__':

    fpath = "/home/rpg/Desktop/rosbags_file/Multiple_Rosbags_cut"
    for(dirpath, dirnames, filenames) in os.walk(fpath):
        rosbags_num = len(filenames)
    for file in filenames:
        bagfile = os.path.join(fpath,file)
        checkValidity(bagfile)
