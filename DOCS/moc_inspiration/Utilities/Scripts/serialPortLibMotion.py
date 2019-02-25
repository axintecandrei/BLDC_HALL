#!/usr/bin/python
# -*- coding: utf-8 -*-
import sys
import threading
import serial
import time

class myThread (threading.Thread):
    def __init__(self, threadID, name):
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.name = name
        self.output = "?"
    def run(self):
		self.output = raw_input()

def open_port_rate_to(port, serialObject, rate, to):
    serialObject.port = 'COM'+port
    serialObject.baudrate = rate
    serialObject.parity = serial.PARITY_NONE
    serialObject.stopbits = serial.STOPBITS_ONE
    serialObject.bytesize = serial.EIGHTBITS
    serialObject.timeout = to
    serialObject.open()
    return serialObject

def write_and_wait_for_string(stringToWrite, awaitString, maxDuration, serialObject, nTrials):
    return write_and_wait_for_string_d(stringToWrite, awaitString, maxDuration, serialObject, nTrials, 1)
    
def write_and_wait_for_string_d(stringToWrite, awaitString, maxDuration, serialObject, nTrials, chardelay):
    maxDuration = int(maxDuration)
    chardelay = float(chardelay)/1000
    for c in stringToWrite:
        serialObject.write(c)
        time.sleep(chardelay)
    from datetime import timedelta
    line = ""
    duration = 0
    trial = 1
    StartTimeInSeconds = time.time()
    while (((awaitString in line) == False) and (duration < maxDuration)):
        duration = (time.time() - StartTimeInSeconds)
        line = serialObject.readline()
        if line == '':
            print 'waiting for string ' + awaitString + ' for ' + str(int(duration)) + ' seconds'
        if ((duration >= maxDuration) and (trial < nTrials)):
            for c in stringToWrite:
                serialObject.write(c)
                time.sleep(0.001)        
            duration = 0
            trial = trial + 1
            StartTimeInSeconds = time.time()
    resultStr = ''
    if (duration < maxDuration):
       resultStr =  line
    else:
       resultStr =  'KO after ' + str(trial) + ' trial(s)'
    return resultStr