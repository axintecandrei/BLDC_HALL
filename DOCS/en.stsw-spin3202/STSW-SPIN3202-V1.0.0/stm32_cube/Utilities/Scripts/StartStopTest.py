import serial
import serialPortLibMotion as ser
import sys
import time

if __name__ == "__main__":
#---Serial port rate------------------------------------------------------------------------------------------------------------#  
  RATE = 230400
#---Serial port number----------------------------------------------------------------------------------------------------------#  
  comport = sys.argv[1]  
#---Number of start stop iterations---------------------------------------------------------------------------------------------#  
  loopMax = int(sys.argv[2])  
#---Target speed----------------------------------------------------------------------------------------------------------------#  
  speed = int(sys.argv[3])
#---Target speed range ---------------------------------------------------------------------------------------------------------#   
  minSpeed = speed - int(sys.argv[4])
  maxSpeed = speed + int(sys.argv[4])
#---Time after stop command in ms-----------------------------------------------------------------------------------------------#     
  stop_time = int(sys.argv[5])
#---Test loop internal variables initialization---------------------------------------------------------------------------------#
  loopIndex = 0
  out = 0
  maxDur = 2
#---Serial port connection------------------------------------------------------------------------------------------------------#
  serialObject = serial.Serial(None)
  ser.open_port_rate_to(comport,serialObject,RATE,1)
#---Start stop loop-------------------------------------------------------------------------------------------------------------#
  while ((speed > minSpeed) and (speed < maxSpeed) and (loopIndex < loopMax) and (out==0)):
#---Start the motor-------------------------------------------------------------------------------------------------------------#
    respStr = ser.write_and_wait_for_string("STARTM\r\n",">> START MOTOR COMMAND RECEIVED ! <<",maxDur,serialObject,1)  
    if (("KO" not in respStr)==False):
      break
    print respStr
    time.sleep(3)
#---Get the motor speed---------------------------------------------------------------------------------------------------------#    
    respStr = ser.write_and_wait_for_string("GETSPD\r\n","Motor Speed:",maxDur,serialObject,3)
    print respStr
    if not("KO" in respStr):
      if (len(respStr.split()) > 2 ):
        speed = int(respStr.split()[2])
      else:
        speed = 0
#---Check that the motor speed in the specified range---------------------------------------------------------------------------#        
      if ((speed < minSpeed) or (speed > maxSpeed)):
        respStr = ser.write_and_wait_for_string("STATUS\r\n","Status:",maxDur,serialObject,3)
        print respStr
      else:
        respStr = ser.write_and_wait_for_string("STOPMT\r\n",">> STOP MOTOR COMMAND RECEIVED ! <<",maxDur,serialObject,3)
        print respStr
        if not("KO" in respStr):
          time.sleep(stop_time)
          loopIndex = loopIndex + 1
          print str(loopIndex) + "\n"
        else:
          out = out + 1
    else:
      out = out + 1
#---Serial port closing---------------------------------------------------------------------------------------------------------#      
  serialObject.close()
#---Print the test result-------------------------------------------------------------------------------------------------------# 
  if ((speed > minSpeed) and (speed < maxSpeed) and (loopIndex == loopMax) and (out==0)):
    print "\nPASS " + str(loopIndex)
  else:
    print "\nFAIL at loop Index = " + str(loopIndex)
