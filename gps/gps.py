import serial
import micropyGPS
import pyproj
#import threading
import time

class GPS_get():
    def _init_(self):
        self.gps = micropyGPS.MicropyGPS(9, 'dd')　#micropyGPSのインスタンス
        self.grs80 = pyproj.Geod(ellps='GRS80') # GRS80楕円体　pyprojのインスタンス

