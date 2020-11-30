import dc_motor
import bno055 as bno
import micropyGPS
import pyproj
import csv
import serial
import threading
import time
import numpy as np

#目標の緯度，経度(ここ自動取得にする)
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

#変数
#最初にゴールのGPSをとる
get_goal_GPS=True
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
pKp=0 #pic比例ゲイン
pKi=0 #pic積分ゲイン
pKd=0 #pic微分ゲイン
lost_paradise=0#見失った時間（とき）
#ぺこーら大回転
maware=0
pre_maware=0

bno055=bno.bno055()#bno055のインスタンス

try:
    while True:
        ###################地上での準備###################
        if get_goal_GPS==True:
            while(gps.clean_sentences < 20):#きちんとした値が20以上とれるまで
                time.sleep(0.5)
            h = gps.timestamp[0] if gps.timestamp[0] < 24 else gps.timestamp[0] - 24
            print('%2d:%02d:%04.1f' % (h, gps.timestamp[1], gps.timestamp[2]))
            print('緯度経度: %2.8f, %2.8f' % (gps.latitude[0], gps.longitude[0]))
            print('海抜: %f' % gps.altitude)
            print(gps.satellites_used)
            print('衛星番号: (仰角, 方位角, SN比)')
            goal_la,goal_lo= gps.latitude[0], gps.longitude[0]
            print('goal_la is {0},goal_lo is {1}'.format(goal_la,goal_lo))          
            time.sleep(0.5)
        #GPSでゴール座標をとる
        #GPS_get_goal()#後で作るze

        ##################################################

        ###################動く準備########################
        accel_zenkai=bno.accel()
        while(accel_zenkai[2]>0):#Z軸の加速度が正（ひっくり返っているとき）
            dc_motor.right(100,1)
            dc_motor.left(100,1)
            time.sleep(3) #3秒間前進

        while(bno055.check()　< 1): #bno055のキャリブレーションステータス確認，1以上でおｋ
            dc_motor.right(100,1)#その場で回転
        own_angle = bno055.angle()　#角度　東0から時計回りで360
        print("angle is {0}".format(own_angle))

        if own_angle is None: #none返したらbno止まってるので前回own_angle使う
            own_angle=preown_angle
            print("bno error preangle is {0}".format(own_angle))    
        #####################################################

        ####################bno角度変換#######################
        if 0 <= own_angle <= 90: #東0から~360なので，北0で右回り～180，左回り～－180の‐180＜0＜180 に補正
            own_angle+=90
        else:
            own_angle-=270
        #####################################################

        if pic_flag==True:#前のループで物体検知できてたら
            if goal_detect()==True:#赤面積80以上でゴール検知
                print("succes")#正常終了
                dc_motor.right(100,0)
                dc_motor.left(100,0)
                dc_motor.cleanup()
                exit()

            pic_error=pic_angle()#画像中のコーンの位置 -1~1
            u=PID(pKp,pKi,pKd,np.abs(pic_error),pic_pre_error,pic_sum_error)
            motor(pic_error,u,threshold,sleep_time)

            if time.time()-first_time>=15*60:
                print("abnormal termination")
                dc_motor.right(100,0) #モータ停止
                dc_motor.left(100,0)
                dc_motor.cleanip()　#clean
                exit()

            #写真をとる
            pic_flag=cone_detect()#物体検知
            if pic_flag==True:
                continue
            lost_paradise=time.time()#見失った時刻
            own_angle = bno055.angle()#9軸に戻るんでもっかいとる
            if 0 <= own_angle <= 90: #東0から~360なので，北0で右回り～180，左回り～－180の‐180＜0＜180 に補正
                own_angle+=90
            else:
                own_angle-=270
            #pre_heading=0
            #heading=pic_error*180

        ###################GPSで制御#########################
        dc_motor.left(100,0) #停止
        dc_motor.right(100,0)
        if GPS_flag == True:#GPSは初回及び3分後のみの回だけ取る
            while(gps.clean_sentences < 20):#きちんとした値が20以上とれるまで
                time.sleep(0.5)
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

            #初回でGPSとったとき
            if once==True:#初回のフラグがTrue
                first_time=time.time()#最初にGPSをとった時刻
                once = False
            last_time=time.time()#最後にGPSを取得した時間

        ###################進行方向等角度変換###################
            if azimuth > 180: #目標角azimuthは北を0として-180~180にする
                azimuth = azimuth-360
            print("azimuth is {0}".format(azimuth))    
            #偏差は
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
            heading=preheading-(own_angle-preown_angle)#GPSとった後は自分の角度差でheadingを更新していく
        #######################################################        

    #PID
        error=np.abs(heading)/180 #偏差(絶対値) 0~1
        u=PID(Kp,Ki,Kd,error,pre_error,sum_error)
    #motor
        motor(heading,u,threshold,sleep_time)#動かす

        preown_angle=own_angle#前回の角度保存
        pre_heading=heading#pre_headingにheadingを代入

        #写真を撮る
        pic_flag=cone_detect() #物体検知したらTrue,無かったらFalse ここで書き換えるのでいいのでは
        if pic_flag==True:
            continue#戦闘に戻る  

        if time.time()-first_time >= 60*15:#15分後に終了
            print("abnormal termination")
            dc_motor.right(100,0) #モータ停止
            dc_motor.left(100,0)
            dc_motor.cleanip()　#clean
            exit()

        if time.time()-lost_paradise>=10:#見失ってから10秒待ってやる!
            dc_motor.right(100,0)
            dc_motor.left(100,0)
            time.sleep(3)
            maware=bno055.angle()
            pre_maware=maware
            while(maware-premaware<=360):
                dc_motor.left(100,1)#回レ
                maware=bno055.angle()
                if(maware-premaware<0):
                    maware=maware+360
                pre_maware=maware
                pic_flag=cone_detect()
                if pic_flag:
                    break    
            if time.time()-first_time >= 60*15:#15分後に終了
                print("abnormal termination")
                dc_motor.right(100,0)
                dc_motor.left(100,0)
                dc_motor.cleanup()
                exit()
            GPS_flag=True
        else:
            #写真を撮る
            pic_flag=cone_detect()
            continue

        if time.time()-last_time >= 60*3:#3分間待ってやる!
            dc_motor.right(100,0)#モータ停止
            dc_motor.left(100,0)
            GPS_flag=True

    #星空の下のdistance_process(meter_unit)
        f = open("datas.csv","a")
        writer = csv.writer(f,lineterminator='\n')
        writer.writerow(["while","azimuth",azimuth,"angle",own_angle,"judge",judge,"heading",heading])
        f.close()

except KeyboardInterrupt:
    dc_motor.cleanup()

###################################
'''
Δ~~~~Δ
ξ ･ェ･ ξ
ξ　~　ξ
ξ　　 ξ
ξ　　 “~～~～〇
ξ　　　　　　 ξ
ξ ξ ξ~～~ξ ξ
　ξ_ξξ_ξ　ξ_ξξ_ξ
　　ヽ(´･ω･)ﾉ
　　　 |　 　/
　　　 U　U
'''