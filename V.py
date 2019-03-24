#! /usr/bin/python3
# -------------------------------------------------------------------------
from serial     import *
from time       import time, sleep
from os         import system
from WiiProxy   import MultiWii
# -------------------------------------------------------------------------
class V(object):
    # ---------------------------------------------------------------------
    c = None
    TAKEOFF_TIME = 10   # in seconds
    LAND_TIME = 10   # in seconds
    HOVER_THROTTLE = 1200
    RATIO = (HOVER_THROTTLE-1000)/TAKEOFF_TIME

    ARM_DELAY    = 0.5
    WRITE_DELAY    = 0.05
    default_motors   = [1000, 1000, 1000, 1000]
    default_channels = [1500, 1500, 1500, 1000]
    channels_arm     = [1500, 1500, 2000, 1000]  #[roll,pitch,yaw,throttle]
    channels_disarm  = [1500, 1500, 1000, 1000]
    msg = None
    # ---------------------------------------------------------------------

    def __init__(self):
        self.s = Serial()
        self.s.port             = "/dev/ttyACM0"
        self.s.baudrate         = 115200
        self.s.bytesize         = EIGHTBITS
        self.s.parity           = PARITY_NONE
        self.s.stopbits         = STOPBITS_ONE
        self.s.write_timeout    = 3
        self.s.xonxoff          = False
        self.s.rtscts           = False
        self.s.dsrdtr           = False
        
        try:
            self.s.open()
        except SerialException:
            print("init serial port error")
            self.msg = "init serial port error"
            #exit()
        #finally:
        #    self.msg = "init serial port error"
        self.c= MultiWii(self.s)
        if not self.c: exit()
        #sleep(MultiWii.INIT_TIMEOUT)

    # ---------------------------------------------------------------------
    def arm(self):
        print("arm")
        start = time()
        elapsed = 0
        while elapsed < self.ARM_DELAY:
            self.set_channels(self.channels_arm)
            sleep(self.WRITE_DELAY)
            elapsed = time() - start

    def disarm(self):
        print("disarm")
        start = time()
        elapsed = 0
        while elapsed < self.ARM_DELAY:
            self.set_channels(self.channels_disarm)
            sleep(MultiWii.WRITE_DELAY)
            elapsed = time() - start

    def close(self):
        self.disarm()
        self.s.close()
        self.s = None

    '''
    #betaflight only (auto_disarm_delay, disarm_kill_switch, small_angle)
    def get_arming(self):
        return self.c.get_arming()
    '''
    ##############################################################################
    def get_altitude(self):
        return self.c.get_altitude()

    def get_attitude(self):
        return self.c.get_attitude()

    def get_motors(self):
        return self.c.get_motors()

    def get_channels(self):
        #return self.c.get_channels(raw=True)  #default raw=False
        return self.c.get_channels()  #default raw=False
        #cleanflight:raw=True / betaflight:raw=False

    def get_imu(self):
        return self.c.get_imu()

    def get_all_data(self):
        imu = self.get_imu()
        att = self.get_attitude()
        alt_raw = self.get_altitude()
        alt = { 'alt': alt_raw[0] }
        #print('alt='+str(alt_raw))
        coord = {**att, **alt}
        motors_list = self.get_motors()
        #print(motors_list)
        motors={ 'm1':motors_list[0],'m2':motors_list[1],'m3':motors_list[2],'m4':motors_list[3] }
        channels=self.get_channels()
        #print(motors_list)
        return {**imu, **coord, **motors, **channels}

    def get_all_data_str(self):
        data = self.get_all_data()
        print(data)
        #accx,accy,accz, gyrx,gyry,gyrz, magx,magy,magz, angx,angy,heading,alt, m1,m2,m3,m4, roll,pitch,yaw,throttle,aux1,aux2,aux3,aux4
        header = ['accy','accz', 'gyrx','gyry','gyrz', 'magx','magy','magz', 'angx','angy','heading','alt', 'm1','m2','m3','m4', 'roll','pitch','yaw','throttle','aux1','aux2','aux3','aux4']  #'accx'
        str0 = str(data["accx"])
        for i in header:
            str0+=','+str(data[i])
        return str0
    ##############################################################################
    def set_motors(self,motors):
        return self.c.set_motors(motors)

    def set_channels(self,channels):
        self.c.set_channels(channels)

    ##############################################################################
    def reset_channels(self):
        self.set_channels(self.default_channels)

    def reset_motors(self):
        self.set_motors(self.default_motors)
    ##############################################################################
    def takeoff_channels(self):
        throttle = 1000+round(self.RATIO*self.TAKEOFF_TIME)
        channels = [1500, 1500, 1500, throttle]
        self.set_channels(channels)
        #print(self.get_channels())
        #print(self.get_motors())

    def land_channels(self):
        throttle = self.HOVER_THROTTLE-round(self.RATIO*self.LAND_TIME)
        channels = [1500, 1500, 1500, throttle]
        self.set_channels(channels)
        #print(self.get_channels())
        #print(self.get_motors())
    '''
    def takeof_motors(self, time_last):
        s = 1000+round(20*time_last)
        motors = [s,s,s,s]
        self.set_motors(motors)
        print(self.get_motors())
    def land_motors(self):
        s = self.HOVER_THROTTLE-round(self.RATIO*self.LAND_TIME)
        motors = [s,s,s,s]
        self.set_motors(motors)
        print(self.get_motors())
    '''
    # ---------------------------------------------------------------------
    def takeoff(self):
        init_time = time()
        time_last = 0
        while time_last < self.TAKEOFF_TIME:
            #print("in takeoff loop, ts=" + str(time_last) +" round="+str(round(time_last)))
            self.takeoff_channels()
            #self.takeof_motors(time_last)
            sleep(0.2)    #send rc > 5hz
            time_last = time()-init_time

    def land(self):
        init_time = time()
        time_last = 0
        while time_last < self.TAKEOFF_TIME:
            #print("in takeoff loop, ts=" + str(time_last) +" round="+str(round(time_last)))
            self.land_channels()
            #self.land_motors(time_last)
            sleep(0.2)    #send rc > 5hz
            time_last = time()-init_time
