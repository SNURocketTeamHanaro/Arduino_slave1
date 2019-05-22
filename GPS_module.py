# GPS pinout
# use USB 2 UART converter
# connect Rx of converter to Tx of GPS
# connect Tx of converter to Rx of GPS
# converter setting: 3v3

import time
import serial
import string
import pynmea2
import RPi.GPIO as gpio


GPS_port = "/dev/ttyUSB0"

GPS_ser = serial.Serial(GPS_port, baudrate = 9600, timeout = 0.5)
i = 0

while i < 7:
    try:
        data = GPS_ser.readline()
    except:
        print("loading")

    if data[0:6] == '$GPGGA':
        msg = pynmea2.parse(data)

        latval = msg.lat
        concatlat = "lat:" + str(latval)
        print concatlat
        longval = msg.lon
        concatlong = "long:" + str(longval)
        print concatlong
        i += 1

time.sleep(0.5)
