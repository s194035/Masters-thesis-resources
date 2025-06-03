#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#This script fits together with "sample_filter_transmit" stm32 project.
#Never use serial.readline()

import serial
import struct
import matplotlib.pyplot as plt
import collections

def init(port, baud: int = 115200):
    ser = serial.Serial(port, baud, timeout = 0.1) #0.1 is the timeout value
    return ser

def de_init(ser: serial.Serial):
    ser.close()

port = "/dev/ttyACM0" #Needs to be looked up using dmesg | grep tty
#port = "/dev/ttyUSB0" #Now we use USB to uart converter
RECIEVE_HEART_RATE = False
DECIMATION_FACTOR = 4
UART_BUFFER_SIZE = int(16/DECIMATION_FACTOR)
if(RECIEVE_HEART_RATE):
        UART_BUFFER_SIZE = UART_BUFFER_SIZE + 1
UART_BYTE_SIZE = int(UART_BUFFER_SIZE * 4)

MAX_SAMPLES = 2000
UPDATE_CONSTANT = 100/DECIMATION_FACTOR
sample_number = 0 #Used for real-time plotting
x_points = collections.deque(maxlen = 1000) #For real-time plotting
y_points = collections.deque(maxlen = 1000) #For real-time plotting
update = 0
heart_rate_counter = 0

filename = "Just testing.txt"
input = int(input("Mode 1 or 2?: "))
match input:
    case 1:
        MAX_SAMPLES = 10000/DECIMATION_FACTOR
    case 2:
        MAX_SAMPLES = 50000/DECIMATION_FACTOR
if(input == 2):
    plt.ion()
    fig, ax = plt.subplots()
    line, = ax.plot([], [], 'b-',linewidth=0.5)


stm = init(port)
stm.flush()
stm.write(bytes([input]))
#For receiving multiple floats and store in a file
with open(filename, 'w') as file:
    counter = 0
    while(counter < MAX_SAMPLES):
        input_number = 1
        data = stm.read(UART_BYTE_SIZE)
        counter = counter + UART_BUFFER_SIZE
        if len(data) != UART_BYTE_SIZE:
            stm.flush()
            continue
        # Partition into chunks of 4 bytes each https://stackoverflow.com/questions/20024490/how-to-split-a-byte-string-into-separate-parts/20024864
        chunks = [data[i:i+4] for i in range(0, len(data), 4)]
        for i, chunk in enumerate(chunks):
            value = struct.unpack('f', chunk)[0] #Convert 4 bytes into their floating point representation
            if(input == 1): #Mode 1 we write to a file
                file.write(f"{value}\n")
                
            if(input == 2): #Mode 2 we plot in real time. This code is taken from chatGPT
                x_points.append(sample_number)
                y_points.append(value)
                update = update + 1
                sample_number = sample_number + 1
                if(update == UPDATE_CONSTANT):
                    update = 0
                    line.set_xdata(x_points)
                    line.set_ydata(y_points)
                    ax.relim()
                    ax.autoscale_view()
                    plt.draw()
                    plt.pause(0.01)
                    """
                    elif(input == 2 and input_number == UART_BUFFER_SIZE):
                        heart_rate_counter = heart_rate_counter + 1
                        if(heart_rate_counter >= 20):
                            heart_rate_counter = 0
                            print(f"Heart rate: {value}\n")
                input_number = input_number + 1
                """
de_init(stm)
#%% ChatGPT generated code to see spectrum and time-domain signal
import numpy as np
import matplotlib.pyplot as plt

# Load data
data = np.loadtxt("Just testing.txt")

data_length = len(data)
fs = 1000  # Sampling frequency
T = 1 / fs * data_length

t = np.arange(0, T, 1 / fs)

# Ensure t has the same length as data
t = t[:data_length]

# Plot time-domain signal
plt.figure()
plt.plot(t, data * 3 / 2**12)
#plt.xlim([0, 0.1])
plt.xlabel("Time [s]")
plt.ylabel("Amplitude")
plt.title("Time-Domain Signal")
plt.show()


plt.figure()
plt.plot(t, data * 3 / 2**12,linewidth=0.5)
plt.xlim([5, 8])
plt.ylim([-0.2,0.2])
plt.xlabel("Time [s]")
plt.ylabel("Amplitude")
plt.title("Time-Domain Signal")
plt.show()

# Frequency analysis
X = np.fft.fftshift(np.fft.fft(data))
delta_f = 1 / T
mag = 1 / data_length * np.abs(X) * 3 / 2**12
freq = np.arange(-fs/2, fs/2, delta_f)

# Ensure freq has the same length as mag
freq = freq[:len(mag)]

plt.figure(100)
plt.plot(freq, mag)
plt.xlabel("Frequency [Hz]")
plt.ylabel("Intensity")
plt.title("Frequency Spectrum")
plt.show()
#%%
de_init(stm)