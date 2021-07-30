
from bagpy import bagreader
import argparse
import os
import rosbag
from numpy import genfromtxt
import re
import pandas as pd
import numpy as np
import shutil



def copyFile(original, target):
    shutil.copyfile(original, target)

def to_keys(text):
    return int(re.split('.csv', text)[0])

def Generate_ts(file, pathCSV):
    col_list = ["t"]
    df = pd.read_csv(os.path.join(pathCSV, file), usecols=col_list)
    ts = np.asarray(df["t"])
    rosbagLength = (ts[-1] - ts[0])
    return rosbagLength

def extract_chunks(file_in, chunks, savePath, fileNumber):
    bagfile = rosbag.Bag(file_in)
    messages = bagfile.get_message_count()
    m_per_chunk = int(round(float(messages) / float(chunks)))
    chunk = 0
    m = 0
    outbag = rosbag.Bag(os.path.join(savePath, str(fileNumber) + "_" + str(chunk) + ".bag"), 'w')

    for topic, msg, t in bagfile.read_messages():
        m += 1
        if m % m_per_chunk == 0:
            outbag.close()
            #copyFile(outbag, os.path.join(savePath, str(fileNumber) + "_" + str(chunk) + ".bag"))
            print("Written Chunk", str(fileNumber) + "." + str(chunk))
            chunk += 1
            outbag = rosbag.Bag(os.path.join(savePath, str(fileNumber) + "_" + str(chunk) + ".bag"), 'w')

        outbag.write(topic, msg, t)
    outbag.close()


if __name__ == '__main__':

    pathBag = "/home/rpg/Desktop/rosbags_file/Multiple_Rosbags_original"
    pathCSV = "/home/rpg/Desktop/rosbags_file/csv/All_renamed"
    savePath = "/home/rpg/Desktop/rosbags_file/Rosbags_Cut_With_Names"
    traj_analysed = 0
    lengthVector = []
    fileNumber = 0
    under40 = 0
    exceeding40 = 0
    exceeding80 = 0


    for(dirpath, dirnames, filenames) in os.walk(pathCSV):
        for file in sorted(filenames, key=to_keys):
            fileNumber = fileNumber + 1
            currentBagfile = os.path.join(pathBag, str(fileNumber) + ".bag")
            print("Analysing", fileNumber)
            rosbagLength = Generate_ts(file, pathCSV)
            lengthVector.append(rosbagLength)
            if rosbagLength <= 40:
                copyFile(currentBagfile, os.path.join(savePath, os.path.basename(currentBagfile)))
                print("No Split", os.path.join(savePath, os.path.basename(currentBagfile)))
                continue
            if rosbagLength > 40 and rosbagLength <= 80:
                currentBagfile = os.path.join(pathBag, str(fileNumber) + ".bag")
                exceeding40 = exceeding40 + 1
                print("Splitting bag in 2 parts:", os.path.basename(currentBagfile), "ts", rosbagLength)
                extract_chunks(currentBagfile, 2, savePath, fileNumber)
            if rosbagLength > 80:
                currentBagfile = os.path.join(pathBag, str(fileNumber) + ".bag")
                exceeding80 = exceeding80 + 1
                print("Splitting bag in 3 parts:", os.path.basename(currentBagfile), "ts", rosbagLength)
                extract_chunks(currentBagfile, 3, savePath, fileNumber)
        break
