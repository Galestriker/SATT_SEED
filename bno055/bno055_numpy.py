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

def quaternion_to_euler_angle_vectorized1(w, x, y, z):
    
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


with open('output_BNO055.csv','w') as f:
        writer = csv.writer(f)
        writer.writerow(['Temperature (degree C)','Accelerometer (m/s^2)','Magnetometer (microteslas)','Gyroscope (rad/sec)','Euler angle','Quaternion','Linear acceleration (m/s^2)','Gravity (m/s^2)'])

quat=[]

while True:
    print("Temperature: {} degrees C".format(sensor.temperature))
    print("Accelerometer (m/s^2): {}".format(sensor.acceleration))
    print("Magnetometer (microteslas): {}".format(sensor.magnetic))
    print("Gyroscope (rad/sec): {}".format(sensor.gyro))
    print("Euler angle: {}".format(sensor.euler))
    print("Quaternion: {}".format(sensor.quaternion))
    print("Linear acceleration (m/s^2): {}".format(sensor.linear_acceleration))
    print("Gravity (m/s^2): {}".format(sensor.gravity))

    quat=sensor.quaternion
    
    if quat[i for i in range(4)] is None:
        continue

    #bool flag = False
    #
    #for i in range(4):
    #    if quat[i] is None:
    #       flag=1
    #
    #if flag==True:
    #    continue'''
    
    quat=quaternion_to_euler_angle_vectorized1(*quat)
    #print(type(quat))
    print("quat to euler: {}".format(quat))
    
    with open('output_BNO055.csv','a') as f:
        writer = csv.writer(f)
        writer.writerow([sensor.temperature,sensor.acceleration,sensor.magnetic,sensor.gyro,sensor.euler,sensor.quaternion,sensor.linear_acceleration,sensor.gravity,quat])

    time.sleep(0.1)


    
    

