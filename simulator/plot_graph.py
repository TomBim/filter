

import math as m
import numpy as np

LOG_FILE = "results/"


file = open(LOG_FILE, "r")
lines = file.readlines()
header = lines.pop(0)
obs_matrix = np.array([])
for line in lines:
    np.append(obs_matrix, np.array(line), axis=0)
    