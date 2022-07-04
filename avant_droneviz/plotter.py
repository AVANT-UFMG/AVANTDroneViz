'''
Read odom data from csv files and plot it.
@Author: AVANT UFMG
'''

import csv
import numpy as np
from matplotlib import pyplot as plt
from matplotlib import animation as ani
from mpl_toolkits import mplot3d
import pandas as pd
from matplotlib.animation import FuncAnimation

def plot_motion(file_path: str,key1: str, key2: str): #file path should be 'velocidades.csv'
    plt.style.use('seaborn')
    fig1, (ax1, ax2) = plt.subplots(nrows=2, ncols=1)
    ani1 = FuncAnimation(fig1, animate(file_path, ax1, ax2, key1,key2), interval=1000)
    plt.tight_layout()
    plt.show()


# TODO
def plot_obstacles():
    pass

# TODO: test 3d animations
def plot_odom(file_path: str, dim: int, title: str = 'Odometry', keep: int = 10, real_time: bool = False, interval: int = 1000) -> None:
    '''
    :param file_path: path of the file containing the data to plot. It is assumed to be [x, y, yaw] or [x, y, z, roll, pitch, yaw].
    :param dim: dimension of the plot. Can be 2D or 3D.
    :param label: label shown in the plot.
    :param keep: number of previous points to keep. If negative, keeps all previous points.
    :param real_time: ``True`` if the plot should be updated in real time.
    :param interval: Animation update interval in ms. Only used if ``real_time`` is ``True``.
    '''

    fig = plt.figure()

    if not 2 <= dim <= 3:
        raise ValueError('Plot dimension must be 2D or 3D.')
        
    if real_time:
        ani_func = __animate if dim == 2 else __animate3d
        animation = ani.FuncAnimation(fig, ani_func, interval=interval, fargs=(file_path, title, 'Path'))
        plt.show()
        return

    data = __read_csv(file_path)
    data_arr = np.array(data)
    data_dim = data_arr.shape[1]
    if dim == 2:
        if not data_dim == 3:
            raise ValueError('Data format must be [x, y, yaw] but dimension is {}.'.format(data_dim))
        
        data_arr = __format_data(data_arr)
        plt.scatter(data_arr[0], data_arr[1], label='Path')
        plt.legend()
        plt.show()

    elif dim == 3:
        if not data_dim == 6:
            raise ValueError('Data format must be [x, y, z, roll, pitch, yaw] but dimension is {}.'.format(data_dim))

        data_arr = __format_data(data_arr)
        ax = plt.axes(projection="3d")
        ax.scatter3d(data_arr[0], data_arr[1], data_arr[2], 'red')
        plt.show()

def animate(file_path,ax1,ax2,key1, key2):
    file = pd.read_csv(file_path)
    linear = file[key1]
    angular = file[key2]

    ax1.cla()
    ax2.cla()

    ax1.plot(linear, label = key1)
    ax2.plot(angular, label = key2)

    ax1.legend(loc='upper left')
    ax2.legend(loc='upper left')

    ax2.set_xlabel('Tempo')

def __animate(i, file_path, title, label: str):
    data = __read_csv(file_path)
    data_arr = __format_data(data)
    if len(data_arr) == 0:
        return

    plt.cla()
    plt.scatter(data_arr[0], data_arr[1], label=label)
    
    plt.xticks(range(int(min(data_arr[0])), int(np.ceil(max(data_arr[0]))+1)))
    plt.yticks(range(int(min(data_arr[1])), int(np.ceil(max(data_arr[1]))+1)))
    plt.title(title)
    plt.axis('equal')
    plt.legend()
    plt.tight_layout()


def __animate3d(i, file_path, label: str):
    data = __read_csv(file_path)
    data_arr = __format_data(data)
    if len(data_arr) == 0:
        return

    ax = plt.axes(projection="3d")
    ax.scatter3d(data_arr[0], data_arr[1], data_arr[2], 'red')
    plt.tight_layout()


def __format_data(data: list):
    return np.transpose(data)


def __read_csv(path: str, save_all: bool = False) -> list:
    ''' 
    Reads ``csv`` file and returns data in a ``list``. 
    
    :param path: path of the ``csv`` file.
    :param savel_all: if ``False`` does not include repeated sequential lines.
    '''

    data = list()
    try:
        with open(path, mode='r') as file:
            csv_file = csv.reader(file)
            prev_line = ''
            for line in csv_file:
                if line != prev_line and not save_all:
                    data.append(line)
                prev_line = line
    except FileNotFoundError as e:
        print('Error reading file {}: {}'.format(path, str(e)))

    return np.array(data, dtype=np.float32)
