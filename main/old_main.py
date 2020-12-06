import dc_motor
import bno055 as bno
import micropyGPS
import pyproj
import csv
import serial
import threading
import time
import numpy as np

#目標の緯度，経度
goal_la = 34.72542167 #latitude
goal_lo = 137.71619667 #longitude

#インスタンス宣言
gps = micropyGPS.MicropyGPS(9, 'dd')　#micropyGPSのインスタンス
grs80 = pyproj.Geod(ellps='GRS80') #GRS80楕円体　pyprojのインスタンス
#モーター制御初期化
dc_motor.setup()

#swichPULLUP->fall なんかの割り込み
#GPIO.setmode(GPIO.BCM)
#GPIO.setup(18,GPIO.IN,pull_up_down=GPIO.PUD_UP)
#GPIO.wait_for_edge(18, GPIO.FALLING)
#print('OK')
#画像でコーン検知する関数は終わりでTrueを無いときFalse

#PID
def PID(Kp,Ki,Kd,error,pre_error,sum_error):#最後逆転してるの注意な
    u=error*Kp+sum_error*Ki+(error-pre_error)*Kd #操作量u,順にpid
    pre_error=error#前回偏差
    sum_error+=error#偏差累積
    if u>1:#u>1の時は丸める
        u=1
    u*=100#0~100にする
    u=np.abs(u-100)#逆転
    return u

def motor(heading,u,threshold,sleep_time):
    #モーター動かすとこ
    if heading > threshold:#インド人を右に
        dc_motor.right(u,1)#右タイヤの回転数を緩める
        dc_motor.left(100,1)
        print("right")
    elif heading < -1*threshold:#インド人を左に
        dc_motor.left(u,1)#左タイヤの回転数を緩める
        dc_motor.right(100,1)
        print("left")
    else:#前進
        dc_motor.right(100,1)
        dc_motor.left(100,1)
        print("forward")
    time.sleep(sleep_time)#ふわふわ時間

#gpsの値を更新するやつ
def rungps(): # GPSモジュールを読み、GPSオブジェクトを更新する
    s = serial.Serial('/dev/ttyUSB0', 9600, timeout=10)
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

#グロ変数
#角度
own_angle = 0 #自分の姿勢角
preown_angle=0 #前回の姿勢角
judge = 0 #目標角との偏角
heading = 0 #向かうべき方角
once = True #初回フラグ
GPS_flag = True#GPSとるかどうか
pre_heading=0 #前回の進行方向角
#PID
pre_error=0 #前回の偏差
sum_error=0 #偏差の累積
Kp=0 #比例ゲイン
Ki=0 #積分ゲイン
Kd=0 #微分ゲイン
#モーター制御
threshold=20#角度閾値
sleep_time=1#ループ時間
#時間経過
first_time=0#GPS初回タイム
last_time=0#GPS最後に取ったタイム
#画像処理
pic_flag=False#物体検知したか
pic_pre_error=0#画像角度の前の偏差
pic_sum_error=0#偏差累積
lost_paradise=0#見失った時間（とき）

bno055=bno.bno055()#bno055のインスタンス

try:
    while True:
        if bno055.check()　< 1:　#bno055のキャリブレーションステータス確認，1以上でおｋ
            dc_motor.right(100,1)#その場で回転
            continue
        own_angle = bno055.angle()　#角度　東0から時計回りで360
        
        if own_angle is None: #none返したらbno止まってるので前回own_angle使う
            own_angle=preown_angle
            print("bno error preangle is {0}".format(own_angle))
        print("angle is {0}".format(own_angle))

        if 0 <= own_angle <= 90: #東0から~360なので，北0で右回り～180，左回り～－180の‐180＜0＜180 に補正
            own_angle+=90
        else:
            own_angle-=270

        if pic_flag=True:#前のループで物体検知できてたら
            if red_are()>=50:#赤面積50以上で
                print("succes")#正常終了
                exit()

            pic_error=pic_angle()#画像中のコーンの位置 -180~180
            u=PID(Kp,Ki,Kd,np.abs(pic_error)/180,pic_pre_error,pic_sum_error)
            motor(pic_error,u,threshold,sleep_time)

            if time.time()-first_time>=15*60:
                print("abnormal termination")
                exit()

            #チェキ
            #pic_flag=cone_detect()#シャニマスはいいぞ
            if pic_flag:
                continue
            pic_flag=False
            lost_paradise=time.time()
            pre_heading=0
            heading=pic_error

        if GPS_flag == True:#GPSは初回及び3分後のみ回だけ取る
            if gps.clean_sentences > 20: # ちゃんとしたデータがある程度たまったら出力する
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
            #初回限定
            if once==True:
                first_time=time.time()
                once = False
            last_time=time.time()

            if azimuth > 180:
                azimuth = azimuth-360
            print("azimuth is {0}".format(azimuth))    
            #進行方向キメ
            judge = azimuth - own_angle

            if judge>180:
                heading = judge - 360 #headingは自分の角度(own_angle)を0としてそこからの角度
            elif judge<-180:
                heading = 360 + judge
            else:
                heading = judge
            print("heading is {0}".format(heading))
            GPS_flag=False
        else:
            heading=preheading-(own_angle-preown_angle)#GPSとった後はheadingを更新していく

        #チェキです
        #pic_flag=cone_detect() 写真判定 コーンあったらTrue,無かったらFalse
        if pic_flag:
            continue#戦闘に戻る          

    #PID
        error=np.abs(heading)/180 #偏差(絶対値) 0~1
        u=PID(Kp,Ki,Kd,error,pre_error,sum_error)
    #motor
        motor(heading,u,threshold,sleep_time)#動かす

        preown_angle=own_angle#前回の角度保存

        if time.time()-first_time >= 60*15:#15分後に終了
            print("abnormal termination")
            exit()

        if time.time()-lost_paradise>=30:#見失ってから30秒待ってやる!
            setsugetsuka=bno055.angle()
            maware=setsugetsuka
            pre_maware=setsugetsuka
            while(maware-setsugetsuka>=360):
                dc_motor.left(100,1)#回レ回レ回レ回レ回レ回レ回レ回レ
                maware=bno055.angle()
                if(maware-premaware<0):
                    maware=maware+360
                pre_maware=maware
                pic_flag=cone_detect()
                if pic_flag:
                    break    
            if time.time()-first_time >= 60*15:#15分後に終了
                print("abnormal termination")
                exit()
            GPS_flag=True
        else:
            #チェキ
            pic_flag=cone_detect()

        if time.time()-last_time >= 60*3:#3分間待ってやる!
            GPS_flag=True

    #星空の下のdistance_process(meter_unit)
        f = open("datas.csv","a")
        writer = csv.writer(f,lineterminator='\n')
        writer.writerow(["while","azimuth",azimuth,"angle",own_angle,"judge",judge,"heading",heading])
        f.close()

except KeyboardInterrupt:
    dc_motor.cleanup()
    