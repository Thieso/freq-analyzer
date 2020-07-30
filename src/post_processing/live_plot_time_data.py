#!/usr/bin/env python
from threading import Thread
from scipy.signal import butter, lfilter, freqz
import serial
import time
import re
import collections
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import struct
import copy

# -----------------------------------------------------------------------------
# This script is only for debugging purposes to see the time values of the
# accelerometer
# -----------------------------------------------------------------------------

class serialPlot:
    def __init__(self, serialPort = '/dev/ttyUSB0', serialBaud = 115200, plotLength = 100, dataNumBytes = 2, numPlots = 1):
        self.port          = serialPort                                               # serial port with the arduino
        self.baud          = serialBaud                                               # baud rate of communication with the arduino
        self.plotMaxLength = plotLength                                               # max number of data points in plot
        self.dataNumBytes  = dataNumBytes                                             # number of data bytes in received value
        self.numPlots      = numPlots                                                 # number of acceleration plots
        self.rawData       = bytearray(4 + numPlots * dataNumBytes)                   # raw data container for the received data
        self.time          = collections.deque([0] * plotLength, maxlen = plotLength) # time value container
        self.data          = []                                                       # data container
        self.isRun         = True                                                     # boolean to communicate closing event with the background thread
        self.isReceiving   = False                                                    # boolean to see if data gets send
        self.thread        = None                                                     # thread for reading data from arduino
        self.plotTimer     = 0                                                        # timer to keep track of plotting time
        self.previousTimer = 0                                                        # previous timer information
        # give an array for each type of data and store them in a list
        for i in range(numPlots):
            self.data.append(collections.deque([0] * plotLength, maxlen=plotLength))
        # connect to arduino over serial
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

    ''' function to prepare acceleration data for plotting '''
    def getSerialData(self, frame, lines, lineValueText, lineLabel, timeText):
        # timer information to see how long the time interval is
        currentTimer = time.perf_counter()
        self.plotTimer = int((currentTimer - self.previousTimer) * 1000)
        self.previousTimer = currentTimer
        timeText.set_text('Plot Interval = ' + str(self.plotTimer) + 'ms')
        # unpack the raw data (first bytes are the time the others are the accel
        # values)
        values = struct.unpack('=Lh', self.rawData)
        # get the time 
        time_value = values[0]
        self.time.append(time_value)
        # get the acceleartion values and set plot specific variables 
        for i in range(self.numPlots):
            data_value = values[i + 1]
            data_value = (data_value / 2048) + 0.34
            self.data[i].append(data_value)
            lines[i].set_data(range(self.plotMaxLength), self.data[i])
            lineValueText[i].set_text('[' + lineLabel[i] + '] = ' + str(data_value))

    ''' background thread for receiving acceleration data '''
    def backgroundThread(self):
        time.sleep(1.0)
        self.serialConnection.reset_input_buffer()
        while (self.isRun):
            self.serialConnection.readinto(self.rawData)
            self.isReceiving = True

    ''' close communcations with the arduino '''
    def close(self):
        self.isRun = False
        self.thread.join()
        self.serialConnection.close()
        print('Disconnected...')

''' main function to run the program '''
def main():
    portName      = '/dev/ttyUSB0'                                              # port to use for arduino communcation
    baudRate      = 115200                                                      # baud rate for communication with arduino
    maxPlotLength = 100                                                         # max data points in plot
    dataNumBytes  = 2                                                           # number of bytes of 1 data point
    numPlots      = 2                                                           # number of plots
    s             = serialPlot(portName, baudRate, maxPlotLength, dataNumBytes) # initializes all required variables

    # start background thread for serial communcation
    s.readSerialStart()

    # Period at which the plot animation updates [ms]
    pltInterval = 50

    # figure and axis for plotting
    xmin = 0
    xmax = maxPlotLength
    ymin = -(10)
    ymax = 10
    fig = plt.figure(figsize=(10, 8))
    ax = plt.axes(xlim=(xmin, xmax), ylim=(float(ymin - (ymax - ymin) / 10), float(ymax + (ymax - ymin) / 10)))
    ax.set_title('Accelerometer (time domain)')
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Acceleration [g]")

    # start the animated plot
    lineLabel = ['Raw', 'Kalman Filtered', 'Z']
    style = ['r-', 'c-', 'b-']
    timeText = ax.text(0.50, 0.95, '', transform=ax.transAxes)
    lines = []
    lineValueText = []
    for i in range(numPlots):
        lines.append(ax.plot([], [], style[i], label=lineLabel[i])[0])
        lineValueText.append(ax.text(0.50, 0.90-i*0.05, '', transform=ax.transAxes))
    anim = animation.FuncAnimation(fig, s.getSerialData, fargs=(lines, lineValueText, lineLabel, timeText), interval=pltInterval)
    plt.legend(loc="upper left")
    plt.show()

    # close the serial plot class
    s.close()

if __name__ == '__main__':
    main()
