import time
import serial
import string
import pynmea2
import threading
import RPi.GPIO as gpio
import rockBlock
from rockBlock import rockBlockProtocol


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
    while i<5:
        data = GPS_ser.readline()
        if data[0:6] == '$GPGGA':
            msg = pynmea2.parse(data)
            latval = str(msg.lat)
            concatlat = "lat:" + str(latval)
            longval = str(msg.lon)
            print data 
            tmp = str(i) + " xx " +latval+" yy "+longval 
            Iridium().write(tmp)
            i += 1

GPS_port = "/dev/ttyUSB0"
GPS_ser = serial.Serial(GPS_port, baudrate = 9600, timeout = 0.5)

t = threading.Thread(target = Subthread, args = (1, 100, GPS_ser))
t.start()

flag = 0
while 1:
    time.sleep(1)
    flag += 1
    print("MainMain")
