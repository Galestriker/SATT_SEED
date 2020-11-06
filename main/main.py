#import time
import move
import bno055 as bno
import micropyGPS
import pyproj
import RPi.GPIO as GPIO
import csv
import serial
import threading

#目標の緯度，経度
goal_la = 34.72542167 #latitude
goal_lo = 137.71619667 #longitude

#インスタンス宣言
gps = micropyGPS.MicropyGPS(9, 'dd')　# micropyGPSのインスタンス
grs80 = pyproj.Geod(ellps='GRS80') # GRS80楕円体　pyprojのインスタンス
#モーター制御初期化
move.setup()

#swichPULLUP->fall なんかの割り込み
#GPIO.setmode(GPIO.BCM)
#GPIO.setup(18,GPIO.IN,pull_up_down=GPIO.PUD_UP)
#GPIO.wait_for_edge(18, GPIO.FALLING)
#print('OK')

#gpsの値を更新するやつ
def rungps(): # GPSモジュールを読み、GPSオブジェクトを更新する
    s = serial.Serial('/dev/serial0', 9600, timeout=10)
    s.readline() # 最初の1行は中途半端なデーターが読めることがあるので、捨てる
    while True:
        sentence = s.readline().decode('utf-8') # GPSデーターを読み、文字列に変換する
        if sentence[0] != '$': # 先頭が'$'でなければ捨てる
            continue
        for x in sentence: # 読んだ文字列を解析してGPSオブジェクトにデーターを追加、更新する
            gps.update(x)

#gpsを裏で動かすスレッド
gpsthread = threading.Thread(target=rungps, args=()) 
#gpsthread = threading.Thread(target=rungps, name="gps", args=(gps,)) # 上の関数を実行するスレッドを生成
gpsthread.daemon = True
gpsthread.start() # スレッドを起動

#グローバル変数
own_angle = 0 #own_azimuth
judge = 0 #目標角との偏角
heading = 0 #向かうべき方角
once = True #初回だけGPSとるフラグ
pre_azimuth=0　#前回の目標角
prejudge=0  #前回の偏角

try:
    while True:
        bno055=bno.bno055()
        if bno055.check()　< 1:　#bno055のキャリブレーションステータス確認，1以上でおｋ
            continue
        own_angle = bno055.angle()　#角度　東0から時計回りで360
        
        if own_angle>=0 and own_angle<=90: #北０で右回り～180，左回り～－180の‐180＜0＜180 に補正
            own_angle+=90
        else:
            own_angle-=270

        if own_angle is None: #none返したらループ
            continue
        print("angle is {0}".format(own_angle))

        if once == True:
            if gps.clean_sentences > 20: # ちゃんとしたデーターがある程度たまったら出力する
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
            once=False
            gpsthread._stop()
        else:
            azimuth =  pre_azimuth - prejudge　#目標角補正

        pre_azimuth = azimuth　#代入しておく
        
        print("azimuth is {0}".format(azimuth))    
    #direct_change
        judge = azimuth - own_angle
        prejudge=judge

        if judge>180:
            heading = judge -360
        elif judge<-180:
            heading = 360 + judge
        else:
            heading = judge
        print("heading is {0}".format(heading))
    #moter_move
        if heading>20:#rightturn
            move.pivot_turn_right()
            time.sleep(0.5)
            print("right")
        elif heading<-20:
            move.pivot_turn_left()
            time.sleep(0.5)
            print("left")
        else:
            move.forward()
            time.sleep(0.5)
            print("move")
    #distance_process(meter_unit)

        f = open("datas.csv","a")
        writer = csv.writer(f,lineterminator='\n')
        writer.writerow(["while","azimuth",azimuth,"angle",own_angle,"judge",judge,"heading",heading])
        f.close()

except KeyboardInterrupt:
    move.stop()
    move.clean()
    #GPIO.cleanup()
    