#!/usr/bin/python
# -*- coding: utf-8 -*-

import RPi.GPIO as GPIO

# ピン設定
left_f = 9 #IN1
left_b = 11 #IN2
right_f = 23 #IN1
right_b = 24 #IN2

#pwmピン設定
#pwmはインスタンスで渡さんといかんのでめんどくさいのでやりません
#pwm_pin0 = 18
#pwm_pin1 = 19

class Move:
   ##_freq = 100000   # PWM周波数 (default)
   ## _range = 1000000   # PWM分解能 (default)

    def __init__(self,rpi):
        self.rpi=rpi
        self.rpi.GPIO.setmode(GPIO.BCM)
        self.rpi.GPIO.setwarnings(False)
        self.rpi.GPIO.setup(right_f, GPIO.OUT)  
        self.rpi.GPIO.setup(right_b,  GPIO.OUT)  
        self.rpi.GPIO.setup(left_f,  GPIO.OUT) 
        self.rpi.GPIO.setup(left_b, GPIO.OUT)

        #self.rpi.GPIO.setup(pwm_pin0, GPIO.OUT)
        #self.rpi.GPIO.setup(pwm_pin1, GPIO.OUT) 

    # 移動関数
    def forward(self):
    #前進
        self.rpi.GPIO.output(right_f, 1)
        self.rpi.GPIO.output(right_b, 0)
        self.rpi.GPIO.output(left_f, 1)
        self.rpi.GPIO.output(left_b, 0)

    def backward(self):
    #後退
        self.rpi.GPIO.output(right_f, 0)
        self.rpi.GPIO.output(right_b, 1)
        self.rpi.GPIO.output(left_f, 0)
        self.rpi.GPIO.output(left_b, 1)
  
    def spin_turn_right(self):
    #右旋回(超信地)
        self.rpi.GPIO.output(right_f, 0)
        self.rpi.GPIO.output(right_b, 1)
        self.rpi.GPIO.output(left_f, 1)
        self.rpi.GPIO.output(left_b, 0)
  
    def spin_turn_left(self):
    #左旋回(超信地)
        self.rpi.GPIO.output(right_f, 1)
        self.rpi.GPIO.output(right_b, 0)
        self.rpi.GPIO.output(left_f, 0)
        self.rpi.GPIO.output(left_b, 1)

    def pivot_turn_right(self):
    #右旋回(信地)
        self.rpi.GPIO.output(right_f, 1)
        self.rpi.GPIO.output(right_b, 1)
        self.rpi.GPIO.output(left_f, 1)
        self.rpi.GPIO.output(left_b, 0)

    def pivot_turn_left(self):
    #左旋回(信地)
        self.rpi.GPIO.output(right_f, 1)
        self.rpi.GPIO.output(right_b, 0)
        self.rpi.GPIO.output(left_f, 1)
        self.rpi.GPIO.output(left_b, 1)

    def spin_turn_right_back(self):
        self.rpi.GPIO.output(right_f, 1)
        self.rpi.GPIO.output(right_b, 0)
        self.rpi.GPIO.output(left_f, 0)
        self.rpi.self.rpi.GPIO.output(left_b, 1)

    def spin_turn_left_back(self):
        self.rpi.GPIO.output(right_f, 0)
        self.rpi.GPIO.output(right_b, 1)
        self.rpi.GPIO.output(left_f, 1)
        self.rpi.GPIO.output(left_b, 0)

    def pivot_turn_right_back(self):
        self.rpi.GPIO.output(right_f, 1)
        self.rpi.GPIO.output(right_b, 1)
        self.rpi.GPIO.output(left_f, 0)
        self.rpi.GPIO.output(left_b, 1)

    def pivot_turn_left_back(self):
        self.rpi.GPIO.output(right_f, 0)
        self.rpi.GPIO.output(right_b, 1)
        self.rpi.GPIO.output(left_f, 1)
        self.rpi.GPIO.output(left_b, 1)
  
    def stop(self):
    #停止
        self.rpi.GPIO.output(right_f, 1)
        self.rpi.GPIO.output(right_b, 1)
        self.rpi.GPIO.output(left_f, 1)
        self.rpi.GPIO.output(left_b, 1)

    def clean(self):
        self.rpi.GPIO.cleanup()

    #def pwm_out(self,pwm,pin,x=100000,y=1000000):#pi.hardware_PWM(self.rpi.GPIO_pin, freq, duty) 
    #    self.pwm=self.rpi.GPIO.PWM(pin, x)
    #    self.pwm.start(y)
    
    #def pwm_change(x,y):
    #    self.rpi.GPIO.pwm.ChangeFrequency(x)
    #    PWM.ChangeDutyCycle(y)

    #def pwm_stop():
    #    pwm.stop()

    