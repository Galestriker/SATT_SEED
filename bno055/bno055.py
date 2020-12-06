import board
import busio
import adafruit_bno055
import numpy as np
#import types
#import time
#import csv

# Use these lines for I2C

#i2c = busio.I2C(board.SCL, board.SDA)
#sensor = adafruit_bno055.BNO055_I2C(i2c)
# User these lines for UART
# uart = busio.UART(board.TX, board.RX)
# sensor = adafruit_bno055.BNO055_UART(uart)

class bno055():
    def _init_(self):
        self._quat=[]
        self.i2c = busio.I2C(board.SCL, board.SDA)
        self.sensor = adafruit_bno055.BNO055_I2C(self.i2c)

    def _quaternion_to_euler_angle_vectorized1(self,w, x, y, z):
        ysqr = y * y

        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + ysqr)
        X = np.degrees(np.arctan2(t0, t1))

        t2 = +2.0 * (w * y - z * x)
        t2 = np.where(t2>+1.0,+1.0,t2)
        #t2 = +1.0 if t2 > +1.0 else t2

        t2 = np.where(t2<-1.0, -1.0, t2)
        #t2 = -1.0 if t2 < -1.0 else t2
        Y = np.degrees(np.arcsin(t2))

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (ysqr + z * z)
        Z = np.degrees(np.arctan2(t3, t4))

        return X, Y, Z 

    def check(self):
        return self.sensor.calibration_status[0] #1>=ならおけ

    def accel(self):
        return self.sensor.acceleration #[x,y,z]の加速度をタプルで返す

    def angle(self):
        self._quat = self.sensor.quaternion
        if  self._quat is None:
            return None
        else:
            return  self._quaternion_to_euler_angle_vectorized1(self._quat)
        '''
        if  self._quat[0] is None or self._quat[1] is None or self._quat[2] is None or self._quat[3] is None:
            return None
        else:
            return  self._quaternion_to_euler_angle_vectorized1(self._quat)
        '''    
