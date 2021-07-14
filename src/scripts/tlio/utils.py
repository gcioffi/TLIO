''' 
Helper for plotting functions
'''

import numpy as np
import yaml


def readDataloaderParams(config_fn):
    with open(config_fn) as file:
        f = yaml.load(file.read())
        #params = {}
    return f

