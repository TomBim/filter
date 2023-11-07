import argparse as argprs
import os
from alive_progress import alive_bar
import math as m
import numpy as np
import matplotlib.pyplot as plt


LOG_FILES_FOLDER = "results/"
LOG_FILES_PREFIX = "20231105-185916"
ENSHORT_RATIO = 41

last_fig_n = [0]


def get_files_names() -> dict[str]:
    """_summary_

    Returns:
        dict[str]: keys:
            'robotEstim'; 'robotTrue'; 'wheelRead'; 'wheelTrue
    """
    return {'robotEstim': LOG_FILES_FOLDER + LOG_FILES_PREFIX + "rEstim.txt",
            'robotTrue': LOG_FILES_FOLDER + LOG_FILES_PREFIX + "rTrue.txt",
            'wheelRead': LOG_FILES_FOLDER + LOG_FILES_PREFIX + "wRead.txt",
            'wheelTrue': LOG_FILES_FOLDER + LOG_FILES_PREFIX + "wTrue.txt"}


def read_file(file_name: str) -> dict[np.array]:
    with open(file_name, "r") as file:
        n_lines = len(file.readlines())

    with open(file_name, "r") as file:
        # first, the header
        header = file.readline().split('\t')
        n_vars = len(header)

        # now the table
        table = np.resize(np.array([], ndmin=2), [0, n_vars])
        with alive_bar(n_lines) as bar:
            for line in file:
                row = np.array([])
                for value in line.split("\t"):
                    row = np.append(row, float(value))
                    bar()
                row = np.column_stack(row)
                table = np.append(table, row, axis=0)
            table = np.transpose(table)

    # now the dictionary
    if n_vars == 4: # n_vars == 4 -> robot's state
        dictio = {"time": table[0],
                  "v": table[1],
                  "vn": table[2],
                  "omega": table[3]}
    else:
        dictio = {"time": table[0],
                  "w1": table[1],
                  "w2": table[2],
                  "w3": table[3],
                  "w4": table[4]}
    
    print("finished reading " + file_name)

    return dictio

            
def read_all_files() -> dict[dict[np.array]]:
    file_names = get_files_names()
    return {'robotEstim': read_file(file_names['robotEstim']),
            'robotTrue': read_file(file_names['robotTrue']),
            'wheelRead': read_file(file_names['wheelRead']),
            'wheelTrue': read_file(file_names['wheelTrue'])}


def plot(data: dict[dict[np.array]], robot_state_bool: bool, last_fig_n: list[int]) -> None:
    last_fig_n[0] += 1
    plt.figure(last_fig_n[0])
    if robot_state_bool:
        time_array = data['robotTrue']['time'] / 1e3

        plt.subplot(311)
        plt.plot(time_array, data['robotEstim']['v'], label='Estimation')
        plt.plot(time_array, data['robotTrue']['v'], label='Ground truth')
        plt.ylabel("v (m/s)")
        plt.xlabel("time (s)")
        plt.legend()

        plt.subplot(312)
        plt.plot(time_array, data['robotEstim']['vn'], label='Estimation')
        plt.plot(time_array, data['robotTrue']['vn'], label='Ground truth')
        plt.ylabel("vn (m/s)")
        plt.xlabel("time (s)")
        plt.legend()

        plt.subplot(313)
        plt.plot(time_array, data['robotEstim']['omega'], label='Estimation')
        plt.plot(time_array, data['robotTrue']['omega'], label='Ground truth')
        plt.ylabel("omega (m/s)")
        plt.xlabel("time (s)")
        plt.legend()
    else:
        time_array = data['wheelTrue']['time'] / 1e3

        plt.subplot(411)
        plt.plot(time_array, data['wheelRead']['w1'], label='Read')
        plt.plot(time_array, data['wheelTrue']['w1'], label='Ground truth')
        plt.title("WHEEL 1")
        plt.ylabel("omega (m/s)")
        plt.xlabel("time (s)")
        plt.legend()
        
        plt.subplot(412)
        plt.plot(time_array, data['wheelRead']['w2'], label='Read')
        plt.plot(time_array, data['wheelTrue']['w2'], label='Ground truth')
        plt.title("WHEEL 2")
        plt.ylabel("omega (m/s)")
        plt.xlabel("time (s)")
        plt.legend()
        
        plt.subplot(413)
        plt.plot(time_array, data['wheelRead']['w3'], label='Read')
        plt.plot(time_array, data['wheelTrue']['w3'], label='Ground truth')
        plt.title("WHEEL 3")
        plt.ylabel("omega (m/s)")
        plt.xlabel("time (s)")
        plt.legend()
        
        plt.subplot(414)
        plt.plot(time_array, data['wheelRead']['w4'], label='Read')
        plt.plot(time_array, data['wheelTrue']['w4'], label='Ground truth')
        plt.title("WHEEL 4")
        plt.ylabel("omega (m/s)")
        plt.xlabel("time (s)")
        plt.legend()

    plt.show(block=False)


def enshort_file(file_name: str, ratio: int) -> None:
    with open(file_name, "r") as file:
        lines = file.readlines()

    # file_name_original = file_name.replace(".txt", "_original.txt")
    # with open(file_name_original, "w") as file:
    #     file.writelines(lines)

    new_lines = []
    n_lines = len(lines)
    # skip header
    new_lines.append(lines[0])
    i = 1
    while i < n_lines:
        new_lines.append(lines[i])
        i += ratio
    

    with open(file_name, "w") as file:
        file.writelines(new_lines)
            
def enshort_all_files(ratio: int) -> None:
    file_names = get_files_names()
    enshort_file(file_names['wheelTrue'], ratio)
    enshort_file(file_names['wheelRead'], ratio)
    enshort_file(file_names['robotEstim'], ratio)
    enshort_file(file_names['robotTrue'], ratio)


def enshort_array(array: np.array, ratio: int) -> np.array:
    array_length = np.size(array)
    new_array_length = (array_length-1) // ratio  +  1
    new_array = np.zeros(new_array_length)
    with alive_bar(new_array_length) as bar:
        for i in range(new_array_length):
            new_array[i] = array[ratio*i]
            bar()
    return new_array


def enshort_data(data: dict[dict[np.array]], ratio: int) -> None:
    if ratio == 1:
        return
    # robot
    robotTrue = data['robotTrue']
    robotEstim = data['robotEstim']
    keys = ['time', 'v', 'vn', 'omega']
    for key in keys:
        print("ROBOT: " + key)
        robotTrue[key] = enshort_array(robotTrue[key], ratio)
        robotEstim[key] = enshort_array(robotEstim[key], ratio)
    
    # wheel
    wheelTrue = data['wheelTrue']
    wheelRead = data['wheelRead']
    keys = ['time', 'w1', 'w2', 'w3', 'w4']
    for key in keys:
        print("WHEEL: " + key)
        wheelTrue[key] = enshort_array(wheelTrue[key], ratio)
        wheelRead[key] = enshort_array(wheelRead[key], ratio)
    


# create the parser
parser = argprs.ArgumentParser()

# define the arguments
parser.add_argument("-p", "--prefix", type=str)
parser.add_argument("-q", "--ratio", type=int)
parser.add_argument("-e", "--enshortFile", type=bool)

# parse the arguments and use them
args = parser.parse_args()
if args.prefix == "last" or args.prefix == None:
    files_list: list[str] = os.listdir("/home/tombim/documents/ita/tg/codes/filter/results/")
    files_list.sort()
    last_log = files_list[-1]
    prefix_last = last_log[0:15]
    LOG_FILES_PREFIX = prefix_last
else:
    LOG_FILES_PREFIX = args.prefix
if not args.ratio == None:
    ENSHORT_RATIO = args.ratio
enshort_files = False
if not args.enshortFile == None:
    if args.enshortFile:
        enshort_files = True

if not enshort_files:
    print("READING FILES")
    data = read_all_files()
    print("ENSHORTING DATA")
    enshort_data(data, ENSHORT_RATIO)
    print("PLOTTING")
    plot(data, True, last_fig_n)
    plot(data, False, last_fig_n)
    plt.show()
    print("THANK YOU! xD")
else:
    print("ENSHORTING FILES")
    enshort_all_files(ENSHORT_RATIO)