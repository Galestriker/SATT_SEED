import pigpio

#ピン指定
RIGHT_MOTOR_GPIO_NUMBER1 = 16
RIGHT_MOTOR_GPIO_NUMBER2 = 20
RIGHT_MOTOR_PWM_GPIO_NUMBER = 12

LEFT_MOTOR_GPIO_NUMBER1 = 5
LEFT_MOTOR_GPIO_NUMBER2 = 6
LEFT_MOTOR_PWM_GPIO_NUMBER = 13

#周波数指定
MOTOR_FREQ = 50

#インスタンスを作成
right_motor1 = pigpio.pi()
right_motor2 = pigpio.pi()
right_motor_pwm = pigpio.pi()
left_motor1 = pigpio.pi()
left_motor2 = pigpio.pi()
left_motor_pwm = pigpio.pi()

#GPIO出力設定
right_motor1.set_mode(RIGHT_MOTOR_GPIO_NUMBER1, pigpio.OUTPUT)
right_motor2.set_mode(RIGHT_MOTOR_GPIO_NUMBER2, pigpio.OUTPUT)
right_motor_pwm.set_mode(RIGHT_MOTOR_PWM_GPIO_NUMBER, pigpio.OUTPUT)
left_motor1.set_mode(LEFT_MOTOR_GPIO_NUMBER1, pigpio.OUTPUT)
left_motor2.set_mode(LEFT_MOTOR_GPIO_NUMBER2,pigpio.OUTPUT)
left_motor_pwm.set_mode(LEFT_MOTOR_PWM_GPIO_NUMBER, pigpio.OUTPUT)


#dcで回転数指定，dirで回転方向指定です．
#dir は 1:CW/CCW  0:STOP  -1:CCW/CW  2:BRAKE
#(0.0 <= dc <= 100.0) speed

def right(dc,dir):
    right_motor_pwm.hardware_PWM(MOTOR_RIGHT_PWM_GPIO_NUMBER, MOTOR_FREQ, int(dc*10000))
    left_motor_pwm.hardware_PWM(MOTOR_LEFT_PWM_GPIO_NUMBER, MOTOR_FREQ, int(dc*10000))
    
    if dir == 1: #CW/CCW
        right_motor1.write(RIGHT_MOTOR_GPIO_NUMBER1, 0)
        right_motor2.write(RIGHT_MOTOR_GPIO_NUMBER2, 1)
    elif dir == 0: #STOP
        right_motor1.write(RIGHT_MOTOR_GPIO_NUMBER1, 0)
        right_motor2.write(RIGHT_MOTOR_GPIO_NUMBER2, 0)
    elif dir == -1: #CCW/CW
        right_motor1.write(RIGHT_MOTOR_GPIO_NUMBER1, 1)
        right_motor2.write(RIGHT_MOTOR_GPIO_NUMBER2, 0)
    elif dir == 2: #BRAKE
        right_motor1.write(RIGHT_MOTOR_GPIO_NUMBER1, 1)
        right_motor2.write(RIGHT_MOTOR_GPIO_NUMBER2, 1)

def left(dc,dir):
    right_motor_pwm.hardware_PWM(RIGHT_MOTOR_PWM_GPIO_NUMBER, MOTOR_FREQ, int(dc*10000))
    left_motor_pwm.hardware_PWM(LEFT_MOTOR_PWM_GPIO_NUMBER, MOTOR_FREQ, int(dc*10000))
    
    if dir == 1: #CW/CCW
        left_motor1.write(LEFT_MOTOR_GPIO_NUMBER1, 0)
        left_motor2.write(LEFT_MOTOR_GPIO_NUMBER2, 1)
    elif dir == 0: #STOP
        left_motor1.write(LEFT_MOTOR_GPIO_NUMBER1, 0)
        left_motor2.write(LEFT_MOTOR_GPIO_NUMBER2, 0)
    elif dir == -1: #CCW/CW
        left_motor1.write(LEFT_MOTOR_GPIO_NUMBER1, 1)
        left_motor2.write(LEFT_MOTOR_GPIO_NUMBER2, 0)
    elif dir == 2: #BRAKE
        left_motor1.write(LEFT_MOTOR_GPIO_NUMBER1, 1)
        left_motor2.write(LEFT_MOTOR_GPIO_NUMBER2, 1)
    
def cleanup():
    right_motor_pwm.hardware_PWM(RIGHT_MOTOR_PWM_GPIO_NUMBER, MOTOR_FREQ, 0)
    left_motor_pwm.hardware_PWM(LEFT_MOTOR_PWM_GPIO_NUMBER, MOTOR_FREQ, 0)
    right_motor1.write(RIGHT_MOTOR_GPIO_NUMBER1, 0)
    right_motor2.write(RIGHT_MOTOR_GPIO_NUMBER2, 0)
    left_motor1.write(LEFT_MOTOR_GPIO_NUMBER1, 0)
    left_motor2.write(LEFT_MOTOR_GPIO_NUMBER2, 0)
    right_motor1.stop()
    right_motor2.stop()
    right_motor_pwm.stop()
    left_motor1.stop()
    left_motor2.stop()
    left_motor_pwm.stop()
