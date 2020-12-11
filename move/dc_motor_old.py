import RPi.GPIO as GPIO

MOTOR_RIGHT_PIN1 = 11
MOTOR_RIGHT_PIN2 = 13
MOTOR_RIGHT_PWM = 32

MOTOR_LEFT_PIN1 = 29
MOTOR_LEFT_PIN2 = 31
MOTOR_LEFT_PWM = 33

def setup():
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(MOTOR_RIGHT_PWM, GPIO.OUT)
    GPIO.setup(MOTOR_LEFT_PWM, GPIO.OUT)

    GPIO.setup(MOTOR_RIGHT_PIN1, GPIO.OUT)
    GPIO.setup(MOTOR_RIGHT_PIN2, GPIO.OUT)

    GPIO.setup(MOTOR_LEFT_PIN1, GPIO.OUT)
    GPIO.setup(MOTOR_LEFT_PIN2, GPIO.OUT)

    r = GPIO.PWM(MOTOR_RIGHT_PWM, 50)
    l = GPIO.PWM(MOTOR_LEFT_PWM, 50)

    r.start(0)
    l.start(0)

#dcで回転数指定，dirで回転方向指定です．
#dir は 1:CW/CCW  0:STOP  -1:CCW/CW  2:BRAKE
#CCW：反時計回り　CW：時計回り
#(0.0 <= dc <= 100.0) speed

def right(dc,dir):
    r.ChangeDutyCycle(dc)
    if dir == 1: #CW/CCW
        GPIO.output(MOTOR_RIGHT_PIN1, GPIO.LOW)
        GPIO.output(MOTOR_RIGHT_PIN2, GPIO.HIGH)
    elif dir == 0: #STOP
        GPIO.output(MOTOR_RIGHT_PIN1, GPIO.LOW)
        GPIO.output(MOTOR_RIGHT_PIN2, GPIO.LOW)
    elif dir == -1: #CCW/CW
        GPIO.output(MOTOR_RIGHT_PIN1, GPIO.HIGH)
        GPIO.output(MOTOR_RIGHT_PIN2, GPIO.LOW)
    elif dir == 2: #BRAKE
        GPIO.output(MOTOR_RIGHT_PIN1, GPIO.HIGH)
        GPIO.output(MOTOR_RIGHT_PIN2, GPIO.HIGH)

def left(dc,dir):
    l.ChangeDutyCycle(dc)
    if dir == 1: #CW/CCW
        GPIO.output(MOTOR_LEFT_PIN1, GPIO.LOW)
        GPIO.output(MOTOR_LEFT_PIN2, GPIO.HIGH)
    elif dir == 0: #STOP
        GPIO.output(MOTOR_LEFT_PIN1, GPIO.LOW)
        GPIO.output(MOTOR_LEFT_PIN2, GPIO.LOW)
    elif dir == -1: #CCW/CW
        GPIO.output(MOTOR_LEFT_PIN1, GPIO.HIGH)
        GPIO.output(MOTOR_LEFT_PIN2, GPIO.LOW)
    elif dir == 2: #BRAKE
        GPIO.output(MOTOR_LEFT_PIN1, GPIO.HIGH)
        GPIO.output(MOTOR_LEFT_PIN2, GPIO.HIGH)
    
def cleanup():
    r.stop()
    l.stop()
    GPIO.cleanup()
