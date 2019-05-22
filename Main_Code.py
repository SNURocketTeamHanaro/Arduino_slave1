###############################################################################
# Filename       : Main_code.py
# Target Machine : Arduino micro  with GY91 sensor module and Iridium com. module
#g 
# Description    : 
#        Decoder of serial input from arduino implementing "Avionics.ino", with
#        default output format of CSV.
#        Main onboard computer after launch, decide the parachute deploy timing,
#        record status and send data via Iridium communication module.
# Usage          : 
#         In command prompt, type "sudo python Main_Code.py". Be aware of the com-
#        munication port. The default is 'COM8'.
#
# Contributor    :
#       rev.1  @  2019.05.17.
#           HANARO, SNU.
#           DeokHyeon Kim, Mingyu Park
###############################################################################
#
#   pinout overview
#
#   |----------------------|
#   |pow              1  2 |
#   |                 3  4 |
#   |                 5  6 |
#   |                 7  8 |
#   |HDMI             9  10|
#   |                 11 12|
#   |                 13 14|
#   |                 15 16|
#   |                 17 18|
#   |                 19 20|
#   |                 21 22|
#   |                 23 24|
#   |                 25 26|
#   |                 27 28|
#   |                 ...  |
#   |                 37 38|
#   |                 39 40|
#   |                      |
#   |                      |
#   |LAN         USB    USB|
#   |----------------------|
#
#
#   GPS module
#       USB-UART converter, 3V3
#       ttyUSB0
#       USB converter TX -> Rx of GPS module, vice versa.
#   
#   Iridium module
#       ttyS0
#       GPIO serial port.
#       GPIO #2(5V) -> 5Vin of Iridium
#       GPIO #6(GND) -> lower board GND of Iridium
#       GPIO #8(Tx of RPi) -> Tx of Iridim
#       GPIO #10(Rx of RPi) -> Rx of Iridium
#
#   Arduino Resetter pin
#       GPIO #37 -> Arduino Resetter pin
#
#   Parachute deploy signal
#       GPIO #33 -> Drogue parachute trigger
#       GPIO #35 -> Main parachute trigger
#
##############################################################################
#
# Imports
import serial
import time
from collections import deque
from datetime import datetime
from ctypes import (Union, Structure, c_uint8, c_uint32, c_float, cast, POINTER)
import sys
#from math import *
import math
import numpy as np
#get data from arduino
# from serial_decoder import *
import RPi.GPIO as GPIO
import string
import pynmea2
import threading
import rockBlock
from rockBlock import rockBlockProtocol

DROGUE = 33
MAIN = 35
ARDRST = 37
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BOARD)
GPIO.setup(ARDRST, GPIO.OUT)

#define CONSTANTs

#Threshold altitude for deployment of parachutes
A_THRESHOLD = 2
DROGUE_PARA_THRESHOLD = 5 
ABS_MAIN_PARA_THRESHOLD = 5  #should be revised
REL_MAIN_PARA_THRESHOLD = 5 #should be revised
ALT_SENSOR_THRESHOLD = 0.7  # compensate
HIGH_ALTITUDE_ALERT = ABS_MAIN_PARA_THRESHOLD + REL_MAIN_PARA_THRESHOLD

#mode bit for main parachute deploy mode
REL_PARA_MODE = 0
ABS_PARA_MODE = 1
para_mode = REL_PARA_MODE

#define state name CONSTANT
DETECT_LAUNCH = 1
ON_FLIGHT = 2
LANDING = 3

#define flight state name CONSTANT
ASCENDING = 0
RESERVED = 1
APOGEE = 2

#define parachute deploy status
NONE = 0
D_OPEN = 1
M_OPEN = 2

#declaration of parameters
state = 1
para_state = 0
flight_state = 0
trigger_apogee = 0

#size of A
a_magnitude = 0
A = 0
altMax = 0


# Mode of operation
#mode_plot = true
#mode_rot_anim = false
#mode_save_data = false


#tilt = 84 * pi / 180

#state_dp = 6
#state_mp = 7

# Field names
#f_dev_time = f_A = np.array([2,3,4])
#f_G = np.array([5,6,7])
#f_temp = 8
#f_press = 9
#f_alt_press = 8
#f_altMax = 9
#f_state = 10

#M = np.random.random((1,10))
# Initialize sate variable
# g
#g = np.array([0, 0, -1])
#g0 = 9.80665

# Sensor to rocket body
#C = np.array([[0, 0, -1], [0, 1 ,0], [1, 0, 0]])
# Inertial frame to Earth frame
#E = np.array([[0, 0, -1], [0, 1, 0], [-1, 0, 0]])

#define file for logging
file_ranNum = np.random.randint(1,100)
file_name = "Logging_data" + str(file_ranNum) + ".txt"

# Connection attributes
port = '/dev/ttyACM0'
GPS_port = '/dev/ttyUSB0'
baud = 115200
GPS_baud = 9600
timeout = 1
packetSize = 36

class Iridium (rockBlockProtocol):
    def write(self, msg):
        rb = rockBlock.rockBlock("/dev/ttyS0", self)
        rb.sendMessage(msg)
        rb.close()
    def rockBlockTxFailed(self):
        print "RockBlockTxFailed"
    
    def rockBlockTxSuccess(self, momsn):
        print "rockBLockTxSuccess " + str(momsn)

def Subthread(ini, top, GPS_ser):
    i = 0
    while 1:
        data = GPS_ser.readline()
        if data[0:6] == '$GPGGA':
            msg = pynmea2.parse(data)
            latval = str(msg.lat)
            concatlat = "lat:" + str(latval)
            longval = str(msg.lon)
            print data
            global state
            global para_state
            global flight_state
            # change i into state variable
            tmp = str(state)+"."+str(para_state)+"."+str(flight_state) + " xx " +latval+" yy "+longval 
            Iridium().write(tmp)
            i += 1

# Data packet type
class DataPacket(Structure):
    _fields_ = [('currentTime', c_uint32),
                ('aCoord', c_float * 3),
                ('gCoord', c_float * 3),
                #('mCoord', c_float * 3),
                #('temp', c_float),
                #('pressure', c_uint32),
                ('alt_press', c_float),
                ('altMax', c_float),
                #('state', c_uint8)
                ]
data = DataPacket()


# Main Code starts from here
# reset arduino and wait for initialization & calibration

print("I want to wake her up")
GPIO.output(ARDRST, True)
time.sleep(2)
GPIO.output(ARDRST, False)
GPIO.cleanup()
time.sleep(10)
# Port
port = '/dev/ttyACM0'
while True:
    try:
        # Connect to the device
        ser = serial.Serial(port, baud, timeout=timeout)
        print('deviceTime' +
            '\taCoord[x]\taCoord[y]\taCoord[z]' +
            '\tgCoord[x]\tgCoord[y]\tgCoord[z]' +
            '\tmCoord[x]\tmCoord[y]\tmCoord[z]' +
            '\talt_press\taltMax')
        # Start collecting data
        break

    except serial.SerialException:
        time.sleep(0.05)
        print("Arduino Connection Failure")

# Initilazing GPS module
# Check wheter GPS is available or not
while True:
    try: 
        GPS_ser = serial.Serial(GPS_port, GPS_baud, timeout = 0.5)
        print('GPS module Turn on')
        break
    except serial.SerialException:
        time.sleep(0.05)
        print("GPS module Connection Failure")

# begin Serial communication begin with arduino
print("I want to write letter")
ser.write('1')  # trigger signal
print("I have sent letter")
while 1:
    print("Wating for love")
    if ser.readable():
        signal = ser.read()
        if signal == '1':
            print("Arduino looked at me!!")
            break;
        # whether Sensors are available or not
        # if sensor is unavailable, Arduino sends 'b' or 'c' as an signal

        elif signal == 'b':
            print("[ERROR] No FABO 9 Axis I2C Brick")
        elif signal == 'c':
            print("[ERROR] No BMP280 (Check Wiring)")

GPIO.cleanup()
GPIO.setmode(GPIO.BOARD)
GPIO.setup(DROGUE, GPIO.OUT)
GPIO.setup(MAIN, GPIO.OUT)      
GPIO.output(DROGUE, False)
GPIO.output(MAIN, False)

# Initialize cache memory
buf = b''
lastbuf = b''
chunk = deque(maxlen=(packetSize + 4))
##
##while 1:
##    if ser.readable():
##                 #print("readable")
##        buf = ser.read(1)
##        print(buf)
# Begin communication
try:
    if ser.readable():
        # Read data with start('$') and end('#') symbol
        print("data read")
        buf = ser.read(2)
        print(buf)
        print("buf read")
        while buf != b'': 
            #print("buf get in woohyo!")
            chunk.append(buf)
            #print(chunk[0])
            # Start and end character matches
            if chunk[0] == b'$' and chunk[1] == b'$' and buf == b'#' and lastbuf == b'#':
                #print("dkdkdkd")
                # Remove start and end symbols
                chunk.pop()
                chunk.pop()
                chunk.popleft()
                chunk.popleft()

                # Cast raw data into a parsable form
                rawData = b''.join(list(chunk))
                data = cast(rawData, POINTER(DataPacket)).contents
                
                #logging data into file
                #with open(file_name, "rw") as data_log: 
                #   data_log.write(data)

                # Parse the data
                currentTime = data.currentTime
            
                aCoord = list(data.aCoord)
                gCoord = list(data.gCoord)
                #mCoord = list(data.mCoord)
                #temperature = float(data.temp)
                #pressure = ((data.pressure / 1024.0 * 5 / 331.0) - 0.004) * (100 / 0.016)
                alt_press = float(data.alt_press)
                dummy_data_altMax = float(data.altMax)
                # We decided not to use altMax from Arduino. 
               
                if state == DETECT_LAUNCH:
                    altMax = alt_press
                    alt_ini = alt_press
                    A = 0
                    A += aCoord[0] * aCoord[0]
                    A += aCoord[1] * aCoord[1]
                    A += aCoord[2] * aCoord[2]
                    a_magnitude = math.sqrt(A)
                    print(a_magnitude)
                    if a_magnitude >= A_THRESHOLD:
                            state = ON_FLIGHT
                            print("I'm on a rocket bro!!!")

                elif state == ON_FLIGHT:
                    if alt_press >= altMax:
                        altMax = alt_press
                    if para_mode == REL_PARA_MODE:
                        #Mode change
                        if alt_press >= ABS_MAIN_PARA_THRESHOLD + REL_MAIN_PARA_THRESHOLD :
                            para_mode = ABS_PARA_MODE
                            print("TOO HIGH!")
                        
                        if flight_state == ASCENDING:
                            if alt_press < altMax - ALT_SENSOR_THRESHOLD:
                                flight_state = RESERVED
                                print("Baby not yet")
                            else: 
                                flight_state = ASCENDING
                                print("No..baby")
                                
                        elif flight_state == RESERVED:
                            if alt_press < altMax - ALT_SENSOR_THRESHOLD:
                                flight_state = APOGEE
                                print("I'm on the top of the world")
                            else: 
                                flight_state = ASCENDING
                                print("More.. Altitude")
                               
                        elif flight_state == APOGEE:
                            print("I'm on Apogee")
                            if altMax - alt_press >= REL_MAIN_PARA_THRESHOLD:
                                GPIO.output(MAIN ,True)
                                para_state = M_OPEN
                                time.sleep(10)
                                state = LANDING
                                print("zzzzz...")
                            elif altMax - alt_press >= DROGUE_PARA_THRESHOLD:
                                print("Drogue Out")
                                GPIO.output(DROGUE,True)
                                time.sleep(10)
                                para_state = D_OPEN
                                print("opened") 
                    
                    elif para_mode ==  ABS_PARA_MODE:
                        if flight_state == ASCENDING:
                            if alt_press < altMax- ALT_SENSOR_THRESHOLD:
                                flight_state = RESERVED
                                print("Baby not yet")
                            else: 
                                flight_state = ASCENDING
                                print("No..baby")
                                
                        elif flight_state == RESERVED:
                            if alt_press < altMax- ALT_SENSOR_THRESHOLD:
                                flight_state = APOGEE
                                print("I'm on the top of the world")
                            else: 
                                flight_state = ASCENDING
                                print("More.. Altitude")
                               
                        elif flight_state == APOGEE:
                            if alt_press - alt_ini <= ABS_MAIN_PARA_THRESHOLD:
                                GPIO.output(MAIN ,True)
                                para_state = M_OPEN
                                time.sleep(10)
                                state = LANDING
                                print("zzzzz...")
                            elif altMax - alt_press >= DROGUE_PARA_THRESHOLD:
                                print("Drogue Out")
                                GPIO.output(DROGUE,True)
                                time.sleep(10)
                                para_state = D_OPEN
                                print("opened") 

                elif state == LANDING:
                    GPIO.cleanup()
                    t = threading.Thread(target = Subthread, args = (1, 100, GPS_ser))
                    t.start()
                    #implement GPS signal

                data_log = open(file_name, "a+")

                data_log.write(str(currentTime) + "\t" + str(aCoord[0]) + "\t" + str(aCoord[1]) + "\t" + str(aCoord[2]) + "\t" +
                    str(gCoord[0]) + "\t" + str(gCoord[1]) + "\t" + str(gCoord[2]) + "\t" +
                    #str(mCoord[0]) + "\t" + str(mCoord[1]) + "\t" + str(mCoord[2]) + "\t" +
                    #str(temperature) + "\t" +
                    # str(pressure) + "\t" +
                    str(alt_press) + "\t" + str(altMax) + "\t" +
                    str(state) + "\t" + str(flight_state) + "\t" + str(para_state) + "\n")

                data_log.close()

                # Print out the data to the MONITOR 
          # Read next byte
            lastbuf = buf
            while 1:
                if ser.readable():
                    #print("readable")
                    buf = ser.read(1)
                    break
                time.sleep(0.05)
        print("sibal")        

except serial.SerialException:
    print("stuck!!")
    sys.stderr.write("SerialException")
    sys.exit(1)

# Close
ser.close()
