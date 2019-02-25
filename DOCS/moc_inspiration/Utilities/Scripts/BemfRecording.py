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
  BEMF_LOOP_TIME = 20 #us
  BEMF_SAMPLES_PER_DMATX = 400
  READ_CHAR_TIMEOUT = 1
  threads = []
  dataYChan = deque()
  dataXChan = deque()
  nsamples = 0
  maxDur = 1
    
#---Setup the measurement-------------------------------------------------------------------------------------------------------#
  serialObject = serial.Serial(None)
  ser.open_port_rate_to(sys.argv[1],serialObject,RATE,1)
  respStr = ser.write_and_wait_for_string_d("MEASEL\r\n","Insert the value",maxDur,serialObject,1,10)
  if (("KO" not in respStr)==False):
    sys.exit()
  respStr = ser.write_and_wait_for_string_d("1\r\n","OK <<",maxDur,serialObject,1,10)
  if (("KO" not in respStr)==False):
    sys.exit()
  respStr = ser.write_and_wait_for_string_d("MEASTA\r\n"," >",maxDur,serialObject,1,10)
  if (("KO" not in respStr)==False):
    sys.exit()
  thread1 = ser.myThread(int(sys.argv[1]), "Reading on port com " + sys.argv[1])
  print "\nPress enter to stop reading on serial port " + sys.argv[1] + "\n\r"
  thread1.daemon = True
  thread1.start()
  
#---Start the motor-------------------------------------------------------------------------------------------------------------#
  respStr = ser.write_and_wait_for_string_d("STARTM\r\n","START MOTOR COMMAND RECEIVED",maxDur,serialObject,1,10)  
  if (("KO" not in respStr)==False):
    sys.exit()
  serialObject.reset_input_buffer()    
  serialObject.close()
  ser.open_port_rate_to(sys.argv[1],serialObject,RATE,READ_CHAR_TIMEOUT)
  
#---Acquire raw samples---------------------------------------------------------------------------------------------------------#
  while (1):
    dataYChan.append(serialObject.read(2))
    if (thread1.isAlive() == False):
      if thread1.output == "":
        print "Recording terminated\n"
        break
      else:
        serialObject.reset_input_buffer()
        serialObject.close()
        ser.open_port_rate_to(sys.argv[1],serialObject,RATE,1)
        serialObject.reset_input_buffer()
        print (ser.write_and_wait_for_string_d((thread1.output).rstrip()+"\r\n",">>",maxDur,serialObject,1,10)).split(">>")[1]
        serialObject.reset_input_buffer()
        serialObject.close()
        ser.open_port_rate_to(sys.argv[1],serialObject,RATE,READ_CHAR_TIMEOUT)
        serialObject.reset_input_buffer()
        thread1 = ser.myThread(int(sys.argv[1]), "Reading on port com " + sys.argv[1])
        thread1.daemon = True
        thread1.start()

#---Stop the motor--------------------------------------------------------------------------------------------------------------#
  serialObject.reset_input_buffer()      
  serialObject.close()
  ser.open_port_rate_to(sys.argv[1],serialObject,RATE,1) 		
  respStr = ser.write_and_wait_for_string_d("STOPMT\r\n",">> STOP MOTOR COMMAND RECEIVED ! <<",maxDur,serialObject,1,10)
  respStr = ser.write_and_wait_for_string_d("MEASTO\r\n","Tx cancelled",maxDur,serialObject,1,10)
  print respStr.split('>')[-1]
  serialObject.flush()  
  serialObject.close()

#---Open a file to store acquired samples---------------------------------------------------------------------------------------#
  try:
    fileHandle = open("bemf.txt","w")
  except IOError:
    print "PYTHON_ERROR: cannot open file: bemf.txt"
  
#---Process the acquired raw samples to extract frame numbers, steps and BEMF values -------------------------------------------#
#   Measurement frame description :
#   [0] 1st start symbol 0xABBA
#   [1] 2nd start symbol 0xCDDC
#   [2] 3rd start symbol 0xEFFE
#   [3] frame number low part
#   [4] frame number high part
#   [5-404] raw sample: Bit 15 Reserved, Bits 14:12 Step, Bits 11:0 BEMF
#   [405] Stop symbol 0xFACE

  tmpBemfDeque = deque()
  tmpStepDeque = deque()
  bemfDeque = deque()
  stepDeque = deque()
  bemfArray = np.array([])
  stepArray = np.array([])
  timeArray = np.array([])
  
  cond=1
  tmp=0
  cnt=0
  nframes=0
  measNumber=0
  corruptedSamples=0
  while(1):
    try:
      while(cond):
        tmp = dataYChan.popleft()
        tmp = ord(tmp[0])+256*ord(tmp[1])
        if (tmp==0xABBA):
          cond=0
        else:
          corruptedSamples += 1
          fileHandle.write("corrupted sample cleared (1): "+str(tmp)+"\n")
      tmp = dataYChan.popleft()
      tmp = ord(tmp[0])+256*ord(tmp[1])
      if (tmp==0xCDDC):
        tmp = dataYChan.popleft()
        tmp = ord(tmp[0])+256*ord(tmp[1])       
        if (tmp==0xEFFE):
          tmp = dataYChan.popleft()
          tmp = ord(tmp[0])+256*ord(tmp[1])
          while (tmp!=0xFACE):
            cnt += 1
            if (cnt==1):
              frameNumber = tmp * 65536
            if (cnt==2):
              frameNumber += tmp
              fileHandle.write("frameNumber: " +str(frameNumber)+"\n")
            if (cnt>2):
              tmpBemfDeque.append(0x0FFF&tmp)
              tmpStepDeque.append((0xF000&tmp)>>12)
            tmp = dataYChan.popleft()
            tmp = ord(tmp[0])+256*ord(tmp[1])
          cond = 1
          if (cnt!=(BEMF_SAMPLES_PER_DMATX+2)):
            corruptedSamples += cnt
            fileHandle.write("Deque cleared of corrupted samples: "+str(cnt)+"\n")
            cnt = 0
            tmpBemfDeque.clear()  
            tmpStepDeque.clear()
          else:
            cnt -= 2
            while (cnt!=0):
              dataXChan.append(frameNumber-cnt)
              cnt -= 1
              tmp = tmpBemfDeque.popleft()
              fileHandle.write(str(tmp)+" - ")
              bemfDeque.append(tmp)
              tmp = tmpStepDeque.popleft()
              fileHandle.write(str(tmp)+"\n")
              stepDeque.append(tmp)
              nsamples += 1            
        else:
          cond = 1
          corruptedSamples += 1
          fileHandle.write("corrupted sample cleared (2): "+str(tmp)+"\n")
      else:
        cond = 1
        corruptedSamples += 1
        fileHandle.write("corrupted sample cleared (3): "+str(tmp)+"\n")
    except IndexError:
      print "Samples received: " + str(nsamples)
      print "Samples corrupted : " + str(corruptedSamples)
      break
  
  fileHandle.close()

#---Plot steps and BEMF values against time-------------------------------------------------------------------------------------#
  
  # Initialise plot
  print('Plotting data...')
    
  fig, (ax1, ax2, ax3, ax4) = plt.subplots(4, sharex=False, sharey=True)
  
  plotindex = 0
  start = plotindex * BEMF_SAMPLES_PER_DMATX 
  stop = (plotindex+1) * BEMF_SAMPLES_PER_DMATX
  timeArray = BEMF_LOOP_TIME * np.asarray(dataXChan)[start:stop:1]
  bemfArray = np.asarray(bemfDeque)[start:stop:1]
  stepArray = np.asarray(stepDeque)[start:stop:1]
  
  # Plot the STEP  
  ax1.set_xlabel('time (us)')
  ax1.plot(timeArray, stepArray, 'b-')
  # Make the y-axis label, ticks and tick labels match the line color.
  ax1.set_ylabel('STEP', color='b')
  ax1.tick_params('y', colors='b')
  
  # Plot the BEMF
  ax1R = ax1.twinx()
  ax1R.plot(timeArray, bemfArray, 'r-')
  # Make the y-axis label, ticks and tick labels match the line color.  
  ax1R.set_ylabel('BEMF', color='r')
  ax1R.tick_params('y', colors='r')

  plotindex = int(((nsamples/BEMF_SAMPLES_PER_DMATX)-1)/2)
  start = plotindex * BEMF_SAMPLES_PER_DMATX 
  stop = (plotindex+1) * BEMF_SAMPLES_PER_DMATX
  timeArray = BEMF_LOOP_TIME * np.asarray(dataXChan)[start:stop:1]
  bemfArray = np.asarray(bemfDeque)[start:stop:1]
  stepArray = np.asarray(stepDeque)[start:stop:1]
  
  # Plot the STEP  
  ax2.set_xlabel('time (us)')
  ax2.plot(timeArray, stepArray, 'b-')
  # Make the y-axis label, ticks and tick labels match the line color.
  ax2.set_ylabel('STEP', color='b')
  ax2.tick_params('y', colors='b')
  
  # Plot the BEMF
  ax2R = ax2.twinx()
  ax2R.plot(timeArray, bemfArray, 'r-')
  # Make the y-axis label, ticks and tick labels match the line color.  
  ax2R.set_ylabel('BEMF', color='r')
  ax2R.tick_params('y', colors='r')

  plotindex = (nsamples/BEMF_SAMPLES_PER_DMATX)-1
  start = plotindex * BEMF_SAMPLES_PER_DMATX 
  stop = (plotindex+1) * BEMF_SAMPLES_PER_DMATX
  timeArray = BEMF_LOOP_TIME * np.asarray(dataXChan)[start:stop:1]
  bemfArray = np.asarray(bemfDeque)[start:stop:1]
  stepArray = np.asarray(stepDeque)[start:stop:1]
  
  # Plot the STEP  
  ax3.set_xlabel('time (us)')
  ax3.plot(timeArray, stepArray, 'b-')
  # Make the y-axis label, ticks and tick labels match the line color.
  ax3.set_ylabel('STEP', color='b')
  ax3.tick_params('y', colors='b')
  
  # Plot the BEMF
  ax3R = ax3.twinx()
  ax3R.plot(timeArray, bemfArray, 'r-')
  # Make the y-axis label, ticks and tick labels match the line color.  
  ax3R.set_ylabel('BEMF', color='r')
  ax3R.tick_params('y', colors='r')  
  
  rebuilt_dataXChan = deque()
  rebuilt_bemfDeque = deque()
  rebuilt_stepDeque = deque()
   
  samplesInserted = 0
  cnt = 0
  tmp = 0
  tmp2 = -1
  while(1):
    try:
      tmp = dataXChan.popleft()
      if ((tmp-tmp2)==1):
        tmp2 = dataXChan.popleft()
        if ((tmp2-tmp)==1):
          rebuilt_dataXChan.append(tmp)
          rebuilt_dataXChan.append(tmp2)
          rebuilt_bemfDeque.append(bemfDeque.popleft())
          rebuilt_bemfDeque.append(bemfDeque.popleft())
          rebuilt_stepDeque.append(stepDeque.popleft())
          rebuilt_stepDeque.append(stepDeque.popleft())
        else:
          rebuilt_dataXChan.append(tmp)
          rebuilt_bemfDeque.append(bemfDeque.popleft())
          rebuilt_stepDeque.append(stepDeque.popleft())            
          while((tmp2-tmp)!=1):
            tmp += 1
            rebuilt_dataXChan.append(tmp)
            rebuilt_bemfDeque.append(-1)
            rebuilt_stepDeque.append(-1)
            samplesInserted += 1      
          rebuilt_dataXChan.append(tmp2)
          rebuilt_bemfDeque.append(bemfDeque.popleft())
          rebuilt_stepDeque.append(stepDeque.popleft())
      else:
        cnt = 0
        while((tmp-tmp2)!=1):
          tmp2 += 1
          cnt += 1
          rebuilt_dataXChan.append(tmp2)
          rebuilt_bemfDeque.append(-1)
          rebuilt_stepDeque.append(-1)
          samplesInserted += 1
        rebuilt_dataXChan.append(tmp)
        rebuilt_bemfDeque.append(bemfDeque.popleft())
        rebuilt_stepDeque.append(stepDeque.popleft())       
        tmp2 += 1
    except IndexError:
      print "Samples inserted: " +str(samplesInserted) +"\n"
      print "Rebuilt done\n"
      break
 
  timeArray = BEMF_LOOP_TIME * np.asarray(rebuilt_dataXChan)
  bemfArray = np.asarray(rebuilt_bemfDeque)
  stepArray = np.asarray(rebuilt_stepDeque)
  
  # Plot the STEP  
  ax4.set_xlabel('time (us)')
  ax4.plot(timeArray, stepArray, 'b-')
  # Make the y-axis label, ticks and tick labels match the line color.
  ax4.set_ylabel('STEP', color='b')
  ax4.tick_params('y', colors='b')
  
  # Plot the BEMF
  ax4R = ax4.twinx()
  ax4R.plot(timeArray, bemfArray, 'r-')
  # Make the y-axis label, ticks and tick labels match the line color.  
  ax4R.set_ylabel('BEMF', color='r')
  ax4R.tick_params('y', colors='r')  

  fig.tight_layout()
  
  plt.show()
  
  