#!/usr/bin/python
# -*- coding: utf-8 -*-

import serial
import serialPortLibMotion as ser
import sys
import time
import threading
from collections import deque   #use for fast append, fast pop (numpy array was too slow)
import numpy as np
import matplotlib.pyplot as plt

if __name__ == "__main__":
  RATE = 230400
  threads = []
  dataYChan = deque()
  dataXChan = deque()
  nsamples = 0
  speed_loop_time = int(sys.argv[2]) #ms
  try:
    xmax = int(sys.argv[3]) #ms
  except BaseException:
    xmax = 0
  try:
    ymax = int(sys.argv[4]) #RPM
  except BaseException:
    ymax = 0
  maxDur = 2
  maxDuration = 30
  
#---Setup the measurement-------------------------------------------------------------------------------------------------------#
  serialObject = serial.Serial(None)
  ser.open_port_rate_to(sys.argv[1],serialObject,RATE,1)
  thread1 = ser.myThread(int(sys.argv[1]), "Reading on port com " + sys.argv[1])
  print "\nPress enter to stop reading on serial port " + sys.argv[1] + "\n\r"
  thread1.daemon = True
  thread1.start()
  time.sleep(0.5)
  respStr = ser.write_and_wait_for_string_d("MEASEL\r\n","Insert the value",maxDur,serialObject,1,10)
  if (("KO" not in respStr)==False):
    sys.exit()
  respStr = ser.write_and_wait_for_string_d("0\r\n","OK <<",maxDur,serialObject,1,10)
  if (("KO" not in respStr)==False):
    sys.exit()
  respStr = ser.write_and_wait_for_string("MEASTA\r\n"," >",maxDur,serialObject,1)
  if (("KO" not in respStr)==False):
    sys.exit()

#---Open a file to store acquired samples---------------------------------------------------------------------------------------#
  try:
    fileHandle = open("speed.txt","w")
  except IOError:
    print "PYTHON_ERROR: cannot open file: speed.txt"
    
#---Start the motor-------------------------------------------------------------------------------------------------------------#
  respStr = ser.write_and_wait_for_string("STARTM\r\n",">> START MOTOR COMMAND RECEIVED ! <<",maxDur,serialObject,1)  
  if (("KO" not in respStr)==False):
    sys.exit()

#---Acquire raw samples---------------------------------------------------------------------------------------------------------#
  from datetime import timedelta
  StartTimeInSeconds = time.time()
  line = ""
  duration = 0
  while ((duration < maxDuration) or (maxDuration == 0)):
    duration = (time.time() - StartTimeInSeconds)
    line = serialObject.readline()
    uline = unicode(line).split('\0')[-1].strip().strip('-')
    if (thread1.isAlive() == False):
      if thread1.output == "":
        print "Recording terminated\n"
        break     
      else:
        print ser.write_and_wait_for_string((thread1.output).rstrip()+"\r\n",">>",maxDur,serialObject,1)
        thread1 = ser.myThread(int(sys.argv[1]), "Reading on port com " + sys.argv[1])
        thread1.daemon = True
        thread1.start()
    if line == '':
      line = serialObject.name + ": timeout reading line in seconds: " + str(int(duration))
      fileHandle.writelines(line +"\n")
    elif (uline.isnumeric() != False):
      fileHandle.writelines(uline+"\n")
      nsamples += 1
      dataXChan.append(nsamples)
      dataYChan.append(int(uline))
      StartTimeInSeconds = time.time()
  fileHandle.close()            

#---Stop the motor--------------------------------------------------------------------------------------------------------------#  
  respStr = ser.write_and_wait_for_string("STOPMT\r\n",">> STOP MOTOR COMMAND RECEIVED ! <<",maxDur,serialObject,1)
  respStr = ser.write_and_wait_for_string("MEASTO\r\n","Tx cancelled",maxDur,serialObject,1)
  print respStr.split('>')[-1]
  print "Samples received: " + str(nsamples)
  serialObject.close()

#---Compute and print speed statistics------------------------------------------------------------------------------------------#
  dataYChanArray = np.asarray(dataYChan)
  if (nsamples>1000):
    print "Average Speed (RPM) : " + str(np.average(dataYChanArray[-1000:-1]))
    print "Standard Speed Deviation (RPM) : " + str(np.std(dataYChanArray[-1000:-1]))	

#---Plot speed against time-----------------------------------------------------------------------------------------------------#
  dataXChanArray = speed_loop_time * np.asarray(dataXChan)
  # Initialise plot
  print('Plotting data...')
  fig = plt.figure()
  plt.xlabel('Time in ms')
  plt.ylabel('Motor Speed')
  myLabel = 'RPM'
  plt.plot(dataXChanArray, dataYChanArray, label = myLabel)
  axes = plt.gca()
  if xmax<>0:
    axes.set_xlim([0,xmax])
  if ymax<>0:
    axes.set_ylim([0,ymax])
  plt.show()
  