#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#This script fits together with "Transmit" stm32 project

import serial
import struct

def init(port, baud: int = 115200):
    ser = serial.Serial(port, baud, timeout = 0.1) #0.1 is the timeout value
    return ser

def de_init(ser: serial.Serial):
    ser.close()

port = "/dev/ttyACM0"
UART_BUFFER_SIZE = 2
UART_BYTE_SIZE = UART_BUFFER_SIZE * 4
filename = "float_values.txt"

stm = init(port)
stm.write(bytes([1]))

#For receiving 1 float data point
#data = stm.readline(4)
#value = struct.unpack('f',data)[0]

#For receiving multiple floats and store in a file
with open(filename, 'w') as file:
    value = 0
    while(value < 101010):
        data = stm.readline(UART_BYTE_SIZE)
        # Partition into chunks of 4 bytes each https://stackoverflow.com/questions/20024490/how-to-split-a-byte-string-into-separate-parts/20024864
        chunks = [data[i:i+4] for i in range(0, len(data), 4)]
        for i, chunk in enumerate(chunks):
            value = struct.unpack('f', chunk)[0] #Convert 4 bytes into their floating point representation
            file.write(f"{value}\n")
de_init(stm)
