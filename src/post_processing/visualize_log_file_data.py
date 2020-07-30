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
from scipy import signal

# set file
file_name = "/home/thies/Code/03_arduino/FrequencyAnalysis/data/analog04.csv"

# initialize list for data from file
data = []

# set maximum and minimum frequency of interest
max_freq = 100
min_freq = 0

# open file
fd = open(file_name, "r")
# first read in sampling time
first_line = fd.readline()
split_first_line = first_line.split(",")
dT = float(split_first_line[1])
dT = dT / 1000000
# read useless second line
fd.readline()
for line in fd:
    data.append(float(line))

# create time vector
t = np.arange(len(data)) * dT
# convert data to numpy array and scale the voltage values to g
data = np.array(data)
data = np.interp(data, [0, 1023*3.3/5], [-200, 200])

# plot time series data
plt.figure()
plt.subplot(311)
plt.plot(t, data)
plt.title('Time Data')
plt.ylabel('Acceleration [g]')
plt.xlabel('Time [sec]')
plt.show()

# perform fft
freq_data = np.abs(np.fft.fft(data)/len(data))
freq_data = 2 * freq_data[0:int(len(freq_data)/2)]
freq_data[0] = 0
f_fft = np.arange(len(freq_data)) / dT / (2 * len(freq_data))

# plot fft of whole time series
plt.subplot(312)
plt.xlim((0, max_freq))
plt.title('FFT')
plt.ylabel('Acceleration [g]')
plt.xlabel('Frequency [Hz]')
plt.plot(f_fft, freq_data)
plt.show()

# perform stft
f, t, Zxx = signal.stft(data, 1/dT, nperseg=5000)
# scale stft to good range to have nice colorbar limits
idx = np.logical_and(f >= min_freq, f <= max_freq)
f = f[idx]
Zxx = Zxx[idx, :]
# plot stft
plt.subplot(313)
plt.contourf(t, f, np.abs(Zxx), np.arange(0, 1.2, .3), extend='both')
plt.ylim((min_freq, max_freq))
plt.title('STFT')
plt.ylabel('Frequency [Hz]')
plt.xlabel('Time [sec]')
# plt.colorbar()
plt.show()
