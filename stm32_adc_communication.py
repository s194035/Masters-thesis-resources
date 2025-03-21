#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#This script fits together with "Sampling" stm32 project.

import serial
import struct

def init(port, baud: int = 57600):
    ser = serial.Serial(port, baud, timeout = 0.1) #0.1 is the timeout value
    return ser

def de_init(ser: serial.Serial):
    ser.close()

port = "/dev/ttyACM0" #Needs to be looked up using dmesg | grep tty
UART_BUFFER_SIZE = 16
UART_BYTE_SIZE = UART_BUFFER_SIZE * 4
MAX_SAMPLES = 4000
filename = "adc_values.txt"


stm = init(port)
stm.flush()
stm.write(bytes([1]))

#For receiving multiple floats and store in a file
with open(filename, 'w') as file:
    counter = 0
    while(counter < MAX_SAMPLES):
        #data = stm.readline(UART_BYTE_SIZE) #Don't use this function. I have spent 6 hours trying to get this damn function to work man.
        data= stm.read(UART_BYTE_SIZE)
        counter = counter + UART_BUFFER_SIZE
        if len(data) != UART_BYTE_SIZE:
            stm.flush()
            continue
        # Partition into chunks of 4 bytes each https://stackoverflow.com/questions/20024490/how-to-split-a-byte-string-into-separate-parts/20024864
        chunks = [data[i:i+4] for i in range(0, len(data), 4)]
        for i, chunk in enumerate(chunks):
            value = struct.unpack('f', chunk)[0] #Convert 4 bytes into their floating point representation
            file.write(f"{value}\n")
de_init(stm)
#%%
import numpy as np
import matplotlib.pyplot as plt

# Load data
data = np.loadtxt("/home/frederik/Python/adc_values.txt")

data_length = len(data)
fs = 1000  # Sampling frequency
T = 1 / fs * data_length

t = np.arange(0, T, 1 / fs)

# Ensure t has the same length as data
t = t[:data_length]

# Plot time-domain signal
plt.figure()
plt.plot(t, data * 3 / 2**12)
plt.xlim([0, 0.1])
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
