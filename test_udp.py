#!/usr/bin/env python3
from modules.pyMultiwii import MultiWii
#from WiiProxy   import MultiWii
import time

def getData(board):
    #data = board.getData(MultiWii.ATTITUDE)  #x,y,heading
    #altitude = board.getData(MultiWii.ALTITUDE)  #estalt
    #data = board.getData(MultiWii.MOTOR)  #m1,m2,m3,m4
    #data = board.getData(MultiWii.RC)  #roll,pitch,yaw
    data = board.getData(MultiWii.RAW_IMU)  #g,a,m
    #pid = board.getData(MultiWii.PID)
    #gps = board.getData(MultiWii.RAW_GPS)
    return data

def setData():
    rcCMD=[1520,1520,1520,1500]
    board.sendCMD(8,MultiWii.SET_RAW_RC,rcCMD)

serialPort = "/dev/ttyACM0"
board = MultiWii(serialPort)
try:
    print("init  getData()")
    print(getData(board))
    board.arm()
    time.sleep(1)
    print("armed getData() x2")
    time.sleep(0.1)
    print(getData(board))
    time.sleep(1)
    print(getData(board))
    time.sleep(1)
    print("armed setData()")
    setData()
    time.sleep(1)
    print("armed getData()")
    print(getData(board))
    print("disarm")
    board.disarm()
    print("disarmed getData()")
    print(getData(board))
except KeyboardInterrupt:
    exit()
