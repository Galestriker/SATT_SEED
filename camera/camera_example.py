# !/usr/bin/env python
# -*- coding: utf-8 -*-

import time
#import picamera
import numpy as np
#import random
import sys
import cv2
import csv

"""
def red_detect(img):
    hsv = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
    hsv_min = np.array([0,50,50])#0,50,0
    hsv_max = np.array([5,255,255])
    mask1 = cv2.inRange(hsv, hsv_min, hsv_max)

    hsv_min = np.array([170,50,50])
    hsv_max = np.array([180,255,255])
    mask2 = cv2.inRange(hsv, hsv_min, hsv_max)
    return mask1 + mask2
"""

def red_detect(img):
"""
赤色検知に必要なCSVを定める関数です．
特にcapture()で使うのでこの関数をプログラムに入れてください．
なお，特に引数と返り値について考えなくても良いです．
"""
    hsv = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
    hsv_min = np.array([0,55,80])#0,50,0
    hsv_max = np.array([5,255,255])
    mask1 = cv2.inRange(hsv, hsv_min, hsv_max)

    hsv_min = np.array([160,110,80])
    hsv_max = np.array([180,255,255])
    mask2 = cv2.inRange(hsv, hsv_min, hsv_max)
    return mask1 + mask2



def capture(i):
"""
写真を撮影して変換して配列を返す関数です．

引数:ループのインクリメントしている変数（無限ループしている設定ですよね？）

返り値：
第一引数：画像の二値化した後の配列（多次元配列）
第二引数：true or false　（対象が画像面積の80%以上ならtrue,それ以外ならfalse)
"""

    #初期設定なので関数内に入れる必要はないです．
    #camera=picamera.PiCamera()
    #camera.resolution(1200,1200)
    #ここから関数

    """
    引数のところに保存する名前を入れてください
    関数内でループをさせていないので，写真を保存するのであれば名前を変える必要があります．
    私が良くやるのはループの回数を名前にする方法です．
    "./"+src(i)+".jpg"などです．

    """
    size=(3200,2400)
    image1=cv2.imread("./hattoricorn/"+str(i)+".jpg")
    #camera.capture("./a.jpg")
    #保存した画像の読み込みです．保存しない方が早いかもしれないので変更してもよいかも
    #image1=cv2.imread("./a.jpg")
    #ノイズ処理です．ただ，csvが割と感度が良いのでいらないかもしれないです．
    image1=cv2.resize(image1,size)
    #image1 = cv2.GaussianBlur(image1, (5, 5), 3)
    #別に定義された関数を用いています．やろうと思えば一つの関数になると思いますが高級関数的な使い方になるのでやめました．
    mono_src = red_detect(image1)
    #0じゃない値の数を数えます．（結局面積を求めていることと同じ．単位はピクセル）
    num = np.count_nonzero(mono_src)
    #画像を保存する関数，引数は保存する名前になる
    cv2.imwrite("./hattorik/"+str(i)+".jpg",mono_src)
    #終了判定
    scale = num/3200/2400
    if scale >=0.8:
        bi = True
    else:
        bi =False

    return mono_src,bi

def convert(array,theta,scale=1.0):
    """
    この関数は0~1の場所を返す関数です．途中で画像の保存などもしています．

    引数：
    第一引数：capture()で返された2次元配列
    第二引数：角度（何もしなければ0°,反時計回りに正な角度をとる．なお，270としても-90の役割を果たす）
    第三引数：何もなければ入力しなくてよい．デフォルト引数にしてある．

    返り値：
    0-1を20等分した値．
    ※配列的には0-19だから19で割っている．
    変更したい場合はnum_devideの数を変えてください．

    """
    num_devide = 20
    h,w=array.shape
    array_pad = np.pad(array,((1300,1300),(900,900)),"constant")
    oy, ox = int(array_pad.shape[0]/2), int(array_pad.shape[1]/2)
    R = cv2.getRotationMatrix2D((ox, oy), theta, scale)
    dst = cv2.warpAffine(array_pad, R, (5000, 5000))    # アフィン変換
    cv2.imwrite("./hattorik/"+str(i)+".png", dst)
    array_sum = dst.sum(axis=0)
    ar1=np.sum(array_sum[:900])
    ar2=array_sum[900:4100]
    ar3=np.sum(array_sum[4100:])
    array_divide = np.array_split(ar2,num_devide)
    array_select = np.array([np.sum(array_divide[i]) for i in range(num_devide)])
    array_select[0] = array_select[0]+ar1
    array_select[num_devide-1] = array_select[num_devide-1]+ar3
    array_index = np.argmax(array_select)
    return array_index/(num_devide-1)



if __name__ == '__main__':
    with open("a.csv","w") as f:
        writer = csv.writer(f, lineterminator='\n')
        for i in range(1,25,1):
            array,judge=capture(i)
            if judge==True:
                print("finish")
            select = convert(array,0)
            writer.writerow([i,select])
