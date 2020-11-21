#!/usr/bin/python
# -*- coding: utf-8 -*-

import RPi.GPIO as GPIO

# ピン設定
left_f = 9 #IN1
left_b = 11 #IN2
right_f = 23 #IN1
right_b = 24 #IN2

#pwmピン設定
#pwm_pin0 = 18
#pwm_pin1 = 19


def setup():
    #GPIO ピン出力設定
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    GPIO.setup(right_f, GPIO.OUT)  
    GPIO.setup(right_b,  GPIO.OUT)  
    GPIO.setup(left_f,  GPIO.OUT) 
    GPIO.setup(left_b, GPIO.OUT)

    #GPIO.setup(pwm_pin0, GPIO.OUT)
    #GPIO.setup(pwm_pin1, GPIO.OUT)  

# 関数
def forward():
#前進
    GPIO.output(right_f, 1)
    GPIO.output(right_b, 0)
    GPIO.output(left_f, 1)
    GPIO.output(left_b, 0)

def backward():
#後退
    GPIO.output(right_f, 0)
    GPIO.output(right_b, 1)
    GPIO.output(left_f, 0)
    GPIO.output(left_b, 1)
  
def spin_turn_right():
#右旋回(超信地)
    GPIO.output(right_f, 0)
    GPIO.output(right_b, 1)
    GPIO.output(left_f, 1)
    GPIO.output(left_b, 0)
  
def spin_turn_left():
#左旋回(超信地)
    GPIO.output(right_f, 1)
    GPIO.output(right_b, 0)
    GPIO.output(left_f, 0)
    GPIO.output(left_b, 1)

def pivot_turn_right():
#右旋回(信地)
    GPIO.output(right_f, 1)
    GPIO.output(right_b, 1)
    GPIO.output(left_f, 1)
    GPIO.output(left_b, 0)

def pivot_turn_left():
#左旋回(信地)
    GPIO.output(right_f, 1)
    GPIO.output(right_b, 0)
    GPIO.output(left_f, 1)
    GPIO.output(left_b, 1)

def spin_turn_right_back():
    GPIO.output(right_f, 1)
    GPIO.output(right_b, 0)
    GPIO.output(left_f, 0)
    GPIO.output(left_b, 1)

def spin_turn_left_back():
    GPIO.output(right_f, 0)
    GPIO.output(right_b, 1)
    GPIO.output(left_f, 1)
    GPIO.output(left_b, 0)

def pivot_turn_right_back():
    GPIO.output(right_f, 1)
    GPIO.output(right_b, 1)
    GPIO.output(left_f, 0)
    GPIO.output(left_b, 1)

def pivot_turn_left_back():
    GPIO.output(right_f, 0)
    GPIO.output(right_b, 1)
    GPIO.output(left_f, 1)
    GPIO.output(left_b, 1)
  
def stop():
#停止
    GPIO.output(right_f, 1)
    GPIO.output(right_b, 1)
    GPIO.output(left_f, 1)
    GPIO.output(left_b, 1)

def clean():
    GPIO.cleanup()

def pwm_out(pin,x=100000,y=1000000):#pi.hardware_PWM(gpio_pin, freq, duty) 
    pwm = GPIO.PWM(pin, x)
    pwm.start(y)
    
def pwm_change(x,y):
    pwm.ChangeFrequency(x)
    pwm.ChangeDutyCycle(y)

def pwm_stop():
    pwm.stop()
