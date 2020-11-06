import time
#import hmc5883l_get_angle
#import compass
import Move
import micropyGPS
import RPi.GPIO as GPIO
import pyproj
import angle
import csv
#import accel
#import angle
import light_get_angle
import serial
import threading

#目標の緯度，経度
goal_la = 34.72542167 #latitude
goal_lo = 137.71619667 #longitude

#インスタンス宣言
Motor = Move(GPIO) #モーター制御のインスタンス
gps = micropyGPS.MicropyGPS(9, 'dd')　#micropyGPSのインスタンス
grs80 = pyproj.Geod(ellps='GRS80') # GRS80楕円体　pyprojのインスタンス

#グローバル変数
own_angle = 0 #own_azimuth
judge = 0 #
heading = 0

#swichPULLUP->fall なんかの割り込み
#GPIO.setmode(GPIO.BCM)
#GPIO.setup(18,GPIO.IN,pull_up_down=GPIO.PUD_UP)
#GPIO.wait_for_edge(18, GPIO.FALLING)
#print('OK')


#gpsを裏で動かすスレッド
gpsthread = threading.Thread(target=rungps, name="gps", args=(gps,)) # 上の関数を実行するスレッドを生成
gpsthread.daemon = True
gpsthread.start() # スレッドを起動

try:
    while True:
    #    angle = angle.get_angle()
        own_angle = angle.get_angle()
        print("angle is {0}".format(own_angle))
    #    if gps.clean_sentences > 20: # ちゃんとしたデーターがある程度たまったら出力する
        h = gps.timestamp[0] if gps.timestamp[0] < 24 else gps.timestamp[0] - 24
        print('%2d:%02d:%04.1f' % (h, gps.timestamp[1], gps.timestamp[2]))
        print('緯度経度: %2.8f, %2.8f' % (gps.latitude[0], gps.longitude[0]))
        print('海抜: %f' % gps.altitude)
        print(gps.satellites_used)
        print('衛星番号: (仰角, 方位角, SN比)')
        own_la,own_lo= gps.latitude[0], gps.longitude[0]
        print('own_la is {0},own_lo is {1}'.format(own_la,own_lo))          
        azimuth, bkw_azimuth, distance = grs80.inv(own_lo, own_la, goal_lo, goal_la)
        print(azimuth, bkw_azimuth, distance)

        time.sleep(0.5)
        print("azimuth is {0}".format(azimuth))
    #direct_change
        judge = azimuth - own_angle
        
        if judge>180:
            heading = judge -360
        elif judge<-180:
            heading = 360 +judge
        else:
            heading = judge
        print("heading is {0}".format(heading))
    #moter_move
        if heading>20:#rightturn
            Motor.right()
            time.sleep(0.5)
            print("right")
        elif heading<-20:
            Motor.left()
            time.sleep(0.5)
            print("left")
        else:
            Motor.move()
            time.sleep(0.5)
            print("move")
    #distance_process(meter_unit)

        f = open("peropero.csv","a")
        writer = csv.writer(f,lineterminator='\n')
        writer.writerow(["while","azimuth",azimuth,"angle",own_angle,"judge",judge,"heading",heading])
        f.close()

except KeyboardInterrupt:
    moter.stop()
    GPIO.cleanup()
    

    


