#!/usr/bin/env python
import serial
import time
import re
import collections
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import scipy.fftpack
import struct
import copy
from scipy.ndimage.interpolation import shift
from threading import Thread, Lock
from scipy import signal

# -----------------------------------------------------------------------------
# Frequency analysis of acceleration signal from acceleration data received over
# serial and with live updating plot of FFT of the signal
# -----------------------------------------------------------------------------

class serialPlot:
    def __init__(self, serialPort = '/dev/ttyUSB0', serialBaud = 115200, dataNumBytes = 2, kalman = 1):
        self.port         = serialPort                                    # serial port with the arduino
        self.baud         = serialBaud                                    # baud rate of communication with the arduino
        self.dataNumBytes = dataNumBytes                                  # number of data bytes of each acceleartion value
        self.rawData      = bytearray(4 + 1 * dataNumBytes)               # raw data from the arduino
        self.isRun        = True                                          # boolean to communicate closing event with the background thread
        self.isReceiving  = False                                         # boolean to see if data gets send
        self.thread       = None                                          # thread for reading data from arduino
        self.thread_data   = None                                          # thread for computing the ffts
        self.newPoints    = 20                                            # number of points at which the plot will be renewed
        self.nfft         = 200                                           # number of points used for fft
        self.nr_ffts      = 20                                            # number of ffts that are shown in the spectrogramm
        self.fftMatrix    = np.zeros((int(self.nfft / 2), self.nr_ffts))  # frequency matrix to hold the ffts
        self.data         = np.zeros(self.nfft * 2).reshape(self.nfft, 2) # data array
        self.time         = np.arange(self.nfft)                          # time array
        self.dT           = 0.005 # sampling time
        self.desired_time = np.arange(0, self.nfft * self.dT, self.dT) # desired time array for interpolated data
        self.total_time   = np.arange(self.nr_ffts)                       # total time array for stft
        self.f            = np.arange(0, 100, 100 / self.nfft / 2)              # frequency vector
        # set data index based on wether filtered data is wanted
        if kalman == 1:
            self.dataID = 2
        else:
            self.dataID = 1

        # start serial connection
        print('Trying to connect to: ' + str(serialPort) + ' at ' + str(serialBaud) + ' BAUD.')
        try:
            self.serialConnection = serial.Serial(serialPort, serialBaud, timeout=4)
            print('Connected to ' + str(serialPort) + ' at ' + str(serialBaud) + ' BAUD.')
        except:
            print("Failed to connect with " + str(serialPort) + ' at ' + str(serialBaud) + ' BAUD.')

    ''' start the thread for reading acceleration data from the arduino and wait
    until data is received '''
    def readSerialStart(self):
        if self.thread == None:
            self.thread = Thread(target=self.backgroundThread)
            self.thread.start()
            # Block till we start receiving values
            while self.isReceiving != True:
                time.sleep(0.1)

    def dataThreadStart(self):
        if self.thread_data == None:
            self.thread_data = Thread(target=self.dataThread)
            self.thread_data.start()

    def dataThread(self):
        # extract the needed data from the received values (either
        # filered or not filtered)
        while (self.isRun):
            for i in range(self.newPoints):
                values = struct.unpack('=Lh', self.rawData)
                tmp = np.arange(2)
                tmp[0] = values[0]
                tmp[1] = values[self.dataID]
                self.data = shift(self.data, [-1, 0])
                self.data[len(self.data)-1, :] = np.transpose(tmp)
                time.sleep(0.0001)
            # get the time values from the data array
            self.time = self.data[:, 0]
            # set initial time to zero
            self.time = self.time - self.time[0]
            # get the acceleration in z from the data array
            tmp_data = (self.data[:, 1] / 2048 ) + 0.34
            # interpolate the data to be evenly spaced
            tmp_data = np.interp(self.desired_time, self.time/1000, tmp_data)
            # perform the fft and limit it to one side
            self.freqData = np.fft.fft(tmp_data)/len(tmp_data)
            self.freqData = np.abs(self.freqData)
            self.freqData = 2 * self.freqData[range(int(len(self.data)/2))]
            # set dc component to zero
            self.freqData[0] = 0
            # convert to log scale if wanted
            # self.freqData = 20 * np.log10(self.freqData)
            # build the frequency vector
            self.f = np.arange(int(len(self.data)/2)) / self.dT / len(self.data)
            # append frequency data (FFT) to the matrix
            self.fftMatrix = shift(self.fftMatrix, [0, -1])
            self.fftMatrix[:, self.fftMatrix.shape[1]-1] = self.freqData
            # TODO build total time vector 
            self.total_time = np.arange(self.nr_ffts)


    ''' function to prepare acceleration data for plotting by doing a fourier
    transformation '''
    def getSerialData(self, frame, plot, ax):
        # set data for plotting
        # plot.remove()
        ax.clear()
        plot = ax.pcolor(self.total_time, self.f, self.fftMatrix, vmin=0, vmax=5)
        return plot

    ''' background thread for reading data from the arduino (acceleration data) '''
    def backgroundThread(self):    # retrieve data
        time.sleep(1.0)  # give some buffer time for retrieving data
        self.serialConnection.reset_input_buffer()
        while (self.isRun):
            self.isReceiving = True
            self.serialConnection.readinto(self.rawData)


    ''' End the program by ending the thread and the serial connection '''
    def close(self):
        self.isRun = False
        self.thread.join()
        self.thread_data.join()
        self.serialConnection.close()
        print('Disconnected...')

''' main function to run the program '''
def main():
    portName     = '/dev/ttyUSB0'                                       # port to use for arduino communcation
    baudRate     = 115200                                               # baud rate for communication with arduino
    dataNumBytes = 2                                                    # number of bytes of 1 acceleration value point
    kalman       = 0                                                    # set kalman filter
    s            = serialPlot(portName, baudRate, dataNumBytes, kalman) # initializes all required variables

    # start background thread for serial communcation
    s.readSerialStart()

    # start thread for fft calculation
    s.dataThreadStart()

    # Period at which the plot animation updates [ms]
    pltInterval = 50

    # figure and axis for plotting
    xmin        = 0   # [Hz]
    xmax        = 100 # [Hz]
    ymin        = 0   # [g]
    ymax        = 1000  # [g]
    fig         = plt.figure(figsize = (10, 8))
    ax          = plt.axes(xlim = (xmin, xmax), ylim = (float(ymin - (ymax - ymin) / 10), float(ymax + (ymax - ymin) / 10)))
    ax.set_title('Accelerometer (frequency domain)')
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Frequency [Hz]")

    # start the animated plot
    lineLabel = ['Z']
    style = 'r-'
    plot = ax.pcolor(np.zeros((s.nfft, s.nr_ffts)))
    anim = animation.FuncAnimation(fig, s.getSerialData, fargs=(plot, ax), interval=pltInterval)
    plt.show()

    # close the serial plot class
    s.close()

if __name__ == '__main__':
    main()
