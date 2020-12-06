import time
from MS5607 import MS5607
import csv
import RPi.GPIO as GPIO

def air_main():
    SERVO_PIN = 12
    XBee_SLEEP_PIN = 15
    ALT1 = 10                   #切り離しを可能にする高度
    ALT2 = 0.2                  #切り離し高度
    TIME1 = 1                   #上昇中のデータ取得間隔
    TIME2 = 0.1                 #落下中のデータ取得間隔
    FREQ = 50                   #PWMの周波数，S0270では100,SGでは50
    ANGLE = 90                  #サーボ回転角度，S0270では45，SG90では90
    ONCE = True
    BACKUP_ALT = 2              #バックアップの切り離し判定高度
    BACKUP_DETACH_TIME = 60     #バックアップの切り離しまでの時間
    filename = 'output_air_main.csv'

    #セットアップ
    print('air_main setup start')
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(XBee_SLEEP_PIN, GPIO.OUT)
    GPIO.output(XBee_SLEEP_PIN, GPIO.LOW)
    print('Xbee sleep off')
    
    GPIO.setup(SERVO_PIN, GPIO.OUT)
    servo = GPIO.PWM(SERVO_PIN, FREQ)
    servo.start(0)

    sensor = MS5607()
    with open(filename,'w') as f:
        writer = csv.writer(f)
        writer.writerow(['Temperature (degree C)','Pressure (hPa)','Altitude (m)'])

    #最初のデータをはじく
    for i in range(3):
        temperature = ms5607.getDigitalTemperature()
        pressure = ms5607.getDigitalPressure()
        default = ms5607.convertPressureTemperature(pressure, temperature)
        tmp = sensor.getTemperature()
        time.sleep(1)

    print("Temperature: {} degrees C".format(tmp))
    print("Default Pressure(hPa): {}".format(converted/100))
    print('air_main setup finish')
    print('Turn the switch.')
    #ここでXbeeのスイッチを切り替え


    input("Are you ready?")
    print('air_main start')
    GPIO.output(XBee_SLEEP_PIN, GPIO.HIGH)
    time.sleep(3)
    print('Check')

    #上昇中
    while True:
        temperature = ms5607.getDigitalTemperature()
        pressure = ms5607.getDigitalPressure()
        converted = ms5607.convertPressureTemperature(pressure, temperature)
        altitude = ms5607.getMetricAltitude(converted, default)
        tmp = sensor.getTemperature()

        with open(filename,'a') as f:
            writer = csv.writer(f)
            writer.writerow([tmp, converted / 100, altitude])
            
        if altitude > ALT1:
            break

        time.sleep(TIME1)
    
    #落下中
    while True:
        temperature = ms5607.getDigitalTemperature()
        pressure = ms5607.getDigitalPressure()
        converted = ms5607.convertPressureTemperature(pressure, temperature)
        altitude = ms5607.getMetricAltitude(converted, default)
        tmp = sensor.getTemperature()

        if altitude < ALT2:
            dc = convert_dc_SG90(float(ANGLE))
            servo.ChangeDutyCycle(dc)
            GPIO.output(XBee_SLEEP_PIN, GPIO.LOW)

            with open(filename,'a') as f:
                writer = csv.writer(f)
                writer.writerow([tmp, converted / 100, altitude])

            time.sleep(3)
            print('Detach')
            break

        with open(filename,'a') as f:
            writer = csv.writer(f)
            writer.writerow([tmp, converted / 100, altitude])

        if altitude < BACKUP_ALT and ONCE == True:
            time_backup = time.time()
            ONCE = False
            
        if　time.time() - time_backup > BACKUP_DETACH_TIME and ONCE == False:
            dc = convert_dc_SG90(float(ANGLE))
            servo.ChangeDutyCycle(dc)
            GPIO.output(XBee_SLEEP_PIN, GPIO.LOW)

            with open(filename,'a') as f:
                writer = csv.writer(f)
                writer.writerow([tmp, converted / 100, altitude])

            time.sleep(3)
            print('Backup detach')
            break

        time.sleep(TIME2)

    servo.stop()
    #本番は全体の最後にcleanupを行う
    #GPIO.cleanup()
    print('air_main finish')


def convert_dc_S0270(deg):
    deg_min = 0.0
    deg_max = 180.0
    dc_min = 5.0
    dc_max = 22.0
    return ((deg - deg_min) * (dc_max - dc_min) / (deg_max - deg_min) + dc_min)


def convert_dc_SG90(deg):
    deg_min = 0.0
    deg_max = 180.0
    dc_min = 2.5
    dc_max = 12.0
    return ((deg - deg_min) * (dc_max - dc_min) / (deg_max - deg_min) + dc_min)


if __name__ == "__main__":
    air_main()
    GPIO.cleanup()
