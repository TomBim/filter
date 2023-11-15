from __future__ import annotations
import argparse as argprs
import os
from alive_progress import alive_bar
import math as m
import numpy as np
import matplotlib.pyplot as plt


LOG_FILES_FOLDER = "results/"
LOG_FILES_PREFIX = "20231105-185916"
ENSHORT_RATIO = 1

last_fig_n = [0]


def get_files_names() -> dict[str]:
    """_summary_

    Returns:
        dict[str]: keys:
            'robotEstim'; 'robotTrue'; 'wheelRead'; 'wheelTrue
    """
    return {'robotEstim': LOG_FILES_FOLDER + LOG_FILES_PREFIX + "rEstim.txt",
            'robotTrue': LOG_FILES_FOLDER + LOG_FILES_PREFIX + "rTrue.txt",
            'wheelEstim': LOG_FILES_FOLDER + LOG_FILES_PREFIX + "wEstim.txt",
            'wheelTrue': LOG_FILES_FOLDER + LOG_FILES_PREFIX + "wTrue.txt",
            'wheelRead': LOG_FILES_FOLDER + LOG_FILES_PREFIX + "wRead.txt"}


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
            'wheelEstim': read_file(file_names['wheelEstim']),
            'wheelTrue': read_file(file_names['wheelTrue']),
            'wheelRead': read_file(file_names['wheelRead'])}


def plot(data: dict[dict[np.array]], robot_state_bool: bool, last_fig_n: list[int]) -> None:
    last_fig_n[0] += 1
    # plt.switch_backend('TkAgg')
    fig = plt.figure(last_fig_n[0])
    if robot_state_bool:
        time_array = data['robotTrue']['time'] / 1e3

        ax1 = fig.add_subplot(311)
        ax2 = fig.add_subplot(312, sharex=ax1)
        ax3 = fig.add_subplot(313, sharex=ax1)
        fig.subplots_adjust(hspace=0)

        ax1.set_title("ROBOT")

        ax1.plot(time_array, data['robotEstim']['v'], label='Estimation')
        ax1.plot(time_array, data['robotTrue']['v'], label='Ground truth')
        ax1.set_ylabel(r'$v$ (m/s)')
        ax1.legend()
        ax1.grid(True)
        ax1.text(1.02, 0.5, r'$v$', transform=ax1.transAxes, fontsize=12)

        ax2.plot(time_array, data['robotEstim']['vn'], label='Estimation')
        ax2.plot(time_array, data['robotTrue']['vn'], label='Ground truth')
        ax2.set_ylabel(r'$v_n$ (m/s)')
        ax2.legend()
        ax2.grid(True)
        ax2.text(1.02, 0.5, r'$v_n$', transform=ax2.transAxes, fontsize=12)

        ax3.plot(time_array, data['robotEstim']['omega'], label='Estimation')
        ax3.plot(time_array, data['robotTrue']['omega'], label='Ground truth')
        ax3.set_xlabel("time (s)")
        ax3.set_ylabel(r'$\omega$ (rad/s)')
        ax3.legend()
        ax3.grid(True)
        ax3.text(1.02, 0.5, r'$\omega$', transform=ax3.transAxes, fontsize=12)
    else:
        time_array = data['wheelTrue']['time'] / 1e3

        ax1 = fig.add_subplot(411)
        ax2 = fig.add_subplot(412, sharex=ax1)
        ax3 = fig.add_subplot(413, sharex=ax1)
        ax4 = fig.add_subplot(414, sharex=ax1)
        fig.subplots_adjust(hspace=0)

        ax1.set_title("WHEELS")

        ax1.plot(time_array, data['wheelRead']['w1'], label='Read')
        ax1.plot(time_array, data['wheelEstim']['w1'], label='Filtered')
        ax1.plot(time_array, data['wheelTrue']['w1'], label='Ground truth')
        ax1.set_ylabel(r'$\omega_1$ (rad/s)')
        ax1.legend()
        ax1.grid(True)
        ax1.text(1.02, 0.5, "WHEEL 1", transform=ax1.transAxes, fontsize=12)      

        ax2.plot(time_array, data['wheelRead']['w2'], label='Read')
        ax2.plot(time_array, data['wheelEstim']['w2'], label='Filtered')
        ax2.plot(time_array, data['wheelTrue']['w2'], label='Ground truth')
        ax2.set_ylabel(r'$\omega_2$ (rad/s)')
        ax2.legend()
        ax2.grid(True)
        ax2.text(1.02, 0.5, "WHEEL 2", transform=ax2.transAxes, fontsize=12)        

        ax3.plot(time_array, data['wheelRead']['w3'], label='Read')
        ax3.plot(time_array, data['wheelEstim']['w3'], label='Filtered')
        ax3.plot(time_array, data['wheelTrue']['w3'], label='Ground truth')
        ax3.set_ylabel(r'$\omega_3$ (rad/s)')
        ax3.legend()
        ax3.grid(True)
        ax3.text(1.02, 0.5, "WHEEL 3", transform=ax3.transAxes, fontsize=12)        

        ax4.plot(time_array, data['wheelRead']['w4'], label='Read')
        ax4.plot(time_array, data['wheelEstim']['w4'], label='Filtered')
        ax4.plot(time_array, data['wheelTrue']['w4'], label='Ground truth')
        ax4.set_xlabel("time (s)")
        ax4.set_ylabel(r'$\omega_4$ (rad/s)')
        ax4.legend()
        ax4.grid(True)
        ax4.text(1.02, 0.5, "WHEEL 4", transform=ax4.transAxes, fontsize=12)

    # fig_manager = plt.get_current_fig_manager()
    # screen_width = fig_manager.window.winfo_screenwidth()
    # screen_height = fig_manager.window.winfo_screenheight()
    # plt.gcf().set_size_inches(screen_width / 100, screen_height / 100)
    fig_file_name = LOG_FILES_FOLDER + LOG_FILES_PREFIX
    if robot_state_bool:
        fig_file_name += "robot"
    else:
        fig_file_name += "wheels"
    plt.show(block=False)
    plt.savefig(fig_file_name + ".pdf")
    plt.savefig(fig_file_name + ".svg")


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
    enshort_file(file_names['wheelEstim'], ratio)
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
    wheelEstim = data['wheelEstim']
    keys = ['time', 'w1', 'w2', 'w3', 'w4']
    for key in keys:
        print("WHEEL: " + key)
        wheelTrue[key] = enshort_array(wheelTrue[key], ratio)
        wheelRead[key] = enshort_array(wheelRead[key], ratio)
        wheelEstim[key] = enshort_array(wheelEstim[key], ratio)
    


# create the parser
parser = argprs.ArgumentParser()

# define the arguments
parser.add_argument("-p", "--prefix", type=str,
                    help="don't use this with '-t'")
parser.add_argument("-q", "--ratio", type=int)
parser.add_argument("-e", "--enshortFile", type=bool)
parser.add_argument("-n", "--noplot", type=bool)
parser.add_argument("-t", "--filterType", type=str, 
                    help="options: 'none', 'KF', 'IF'")
parser.add_argument("-v", "--filterVersion", type=str,
                    help="options: '1.0', '2.0'")


# parse the arguments and use them
args = parser.parse_args()
if args.prefix == "last" or args.prefix == None:
    pathzera = "/home/tombim/documents/ita/tg/codes/filter/results/"
    
    # select the type of filter
    if args.filterType is not None:
        # if it's passed through cmd line... we 
        # just accept it xD

        if args.filterType == "none":
            pathzera += "noFilter/"
        elif args.filterType == "KF":
            if (args.filterVersion is not None) and (args.filterVersion == "2.0"):
                pathzera += "KFv2/"
            else:
                pathzera += "KFv1/"
        elif args.filterType == "IF":
            if (args.filterVersion is not None) and (args.filterVersion == "2.0"):
                pathzera += "IFv2/"
            else:
                pathzera += "IFv1/"
    else:
        # else, we need to find the last one modified
        files_list: list[str] = os.listdir(pathzera)
        files_list.sort()
        last_folder = files_list[-1]
        last_modified_folder = last_folder[0:15]
        pathzera += last_modified_folder + "/"
    
    LOG_FILES_FOLDER = pathzera
    
    # find the last modified files
    files_list: list[str] = os.listdir(pathzera)
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
SHOW_PLOT = True
if args.noplot is not None:
    SHOW_PLOT = not args.noplot

if not enshort_files:
    print("READING FILES")
    data = read_all_files()
    print("ENSHORTING DATA")
    enshort_data(data, ENSHORT_RATIO)
    print("PLOTTING")
    plot(data, True, last_fig_n)
    plot(data, False, last_fig_n)
    if SHOW_PLOT:
        plt.show()
    print("THANK YOU! xD")
else:
    print("ENSHORTING FILES")
    enshort_all_files(ENSHORT_RATIO)