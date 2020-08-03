#!/usr/bin/env python
import serial
import time
import re
import collections
import numpy as np
import matplotlib.pyplot as plt
import scipy.fftpack
import struct
import copy
import scipy.integrate as integrate
import os
from scipy import signal

# set file
file_name = "../../data/analog08.csv"

# function to convert measurements to g
def convert_to_g(data_analog):
    data_g = ((data_analog - 0.5 * 1023 * 3.3/5) / (1023 * 3.3 / 5) ) * 200
    return data_g

# number of fft points used in the stft
nfft = 1000

# initialize list for data from file
dataZ = []
dataY = []
dataX = []

# set maximum and minimum frequency of interest
max_freq = 250
min_freq = 5

# open file
abs_path = os.path.dirname(__file__)
fd = open(os.path.join(abs_path, file_name), "r")
# first read in sampling time
first_line = fd.readline()
split_first_line = first_line.split(",")
dT = float(split_first_line[1])
dT = dT / 1000000
# read useless second line
fd.readline()
for line in fd:
    split_line = line.split(",")
    dataZ.append(float(split_line[0]))
    dataY.append(float(split_line[1]))
    dataX.append(float(split_line[2]))

# convert data to numpy array 
dataZ = convert_to_g(np.array(dataZ))
dataY = convert_to_g(np.array(dataY))
dataX = convert_to_g(np.array(dataX))
dataT = np.sqrt(dataZ ** 2 + dataY ** 2 + dataX ** 2)
data = np.array([dataZ, dataY, dataX, dataT])
std_data = np.std(data, axis=1)
# create time vector
t = np.arange(np.size(data, 1)) * dT

# define names for data sources
data_source = ["Z", "Y", "X", "Total"]

for i in range(np.size(data, 0)):
    # get relevant data
    accel_data = data[i, :]
    # plot time series data
    fig = plt.figure(i)
    plt.subplot(211)
    plt.plot(t, accel_data)
    plt.xlim((0, max(t)))
    plt.title('Time Data [' + data_source[i] + ']')
    plt.ylabel('Acceleration [g]')
    plt.xlabel('Time [sec]')

    # perform stft
    f_stft, t_stft, Zxx = signal.stft(accel_data, 1/dT, nperseg=nfft)
    # scale stft to good range to have nice colorbar limits
    idx = np.logical_and(f_stft >= min_freq, f_stft <= max_freq)
    f_stft = f_stft[idx]
    Zxx = Zxx[idx, :]
    # plot stft
    plt.subplot(212)
    plt.contourf(t_stft, f_stft, np.abs(Zxx), np.arange(0, 0.9, .1), extend='both', cmap='rainbow')
    # plt.contourf(t_stft, f_stft, np.abs(Zxx))
    plt.ylim((min_freq, max_freq))
    plt.title('STFT [' + data_source[i] + ']')
    plt.ylabel('Frequency [Hz]')
    plt.xlabel('Time [sec]')
    plt.tight_layout()
    # plt.colorbar()
    plt.show()
    # save figure to folder
    fig.savefig('visualized_data_' + data_source[i] + '.jpg')
