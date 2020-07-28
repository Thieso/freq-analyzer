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
file_name = "/home/thies/Code/03_arduino/FrequencyAnalysis/src/post_processing/DATA04.CSV"

# initialize lists for data from file
t = []
data = []

# set maximum frequency of interest
max_freq = 100

# open file
fd = open(file_name, "r")
for line in fd:
    split_line = line.split(",")
    t.append(float(split_line[0]))
    data.append(float(split_line[1]))

# convert to numpy arrays
t = np.array(t)
t = t/1000
data = (np.array(data) / 2048) + 0.34

# interpolate the data to have equally spaced samples
t_inter = np.linspace(t[0], max(t), len(t))
data_inter = np.interp(t_inter, t, data)

# plot original and interpolated data
plt.figure()
plt.plot(t, data, label='Original')
plt.plot(t_inter, data_inter, label='Interpolated')
plt.title('Time Data')
plt.ylabel('Acceleration [g]')
plt.xlabel('Time [sec]')
plt.legend()
plt.show()

# set t and data to the interpolated values
t = t_inter
data = data_inter

# compute sampling time
dT = max(t)/len(t)

# perform fft
freq_data = np.abs(np.fft.fft(data)/len(data))
freq_data = 2 * freq_data[0:int(len(freq_data)/2)]
freq_data[0] = 0
f_fft = np.arange(len(freq_data)) / dT / (2 * len(freq_data))
plt.figure()
plt.xlim((0, max_freq))
plt.title('FFT')
plt.ylabel('Acceleration [g]')
plt.xlabel('Frequency [Hz]')
plt.plot(f_fft, freq_data)
plt.show()

# perform stft
f, t, Zxx = signal.stft(data, 1/dT, nperseg=200)
plt.figure()
plt.pcolormesh(t, f, np.abs(Zxx), shading='gouraud')
plt.ylim((0, max_freq))
plt.title('STFT')
plt.ylabel('Frequency [Hz]')
plt.xlabel('Time [sec]')
plt.colorbar()
plt.show()