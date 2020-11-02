import time
import board
import busio
import adafruit_bno055
import csv
import numpy as np
import types

# Use these lines for I2C
i2c = busio.I2C(board.SCL, board.SDA)
sensor = adafruit_bno055.BNO055_I2C(i2c)
# User these lines for UART
# uart = busio.UART(board.TX, board.RX)
# sensor = adafruit_bno055.BNO055_UART(uart)

class bno055:
    def _init_(self):
        self.__quat :float=[]

    def __quaternion_to_euler_angle_vectorized1(w, x, y, z):
    
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

    def check:
        return sensor.calibration_status[0]

    def bno055:
        self.__quat=sensor.quaternion
        if  quat[i for i in range(4)] is None:
            return None
        else:
            return  __quaternion_to_euler_angle_vectorized1(self.__quat)
