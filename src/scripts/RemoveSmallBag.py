
from bagpy import bagreader
import argparse
import os
import rosbag
from numpy import genfromtxt
import re
import pandas as pd
import numpy as np
import shutil


def to_keys(text):
    return int(re.split('.bag', text)[0])



if __name__ == '__main__':

    Path = "/home/rpg/Desktop/rosbags_file/Rosbags_Cut_With_Names"

    for(dirpath, dirnames, filenames) in os.walk(Path):
        print(sorted(filenames, key=to_keys))
        for file in sorted(filenames, key=to_keys):
            bagfile = rosbag.Bag(os.path.join(Path, file))
            messages = bagfile.get_message_count()
            print(file, messages)
            #Look at the numbers and remove small bags

        break
