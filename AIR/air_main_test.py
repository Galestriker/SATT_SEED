import time
import MS5607
import csv
import pigpio

def air_main():
    SERVO_GPIO_NUMBER = 19
    XBee_SLEEP_GPIO_NUMBER = 18
    ALT1 = 2                   #切り離しを可能にする高度
    ALT2 = 0.2                  #切り離し高度
    TIME1 = 1                   #上昇中のデータ取得間隔
    TIME2 = 0.1                 #落下中のデータ取得間隔
    FREQ = 50                   #PWMの周波数，S0270では100,SGでは50
    ANGLE = 90                  #サーボ回転角度，S0270では45，SG90では90
    ONCE = True
    BACKUP_ALT = 1              #バックアップの切り離し判定高度
    BACKUP_DETACH_TIME = 60     #バックアップの切り離しまでの時間
    filename = 'output_air_main.csv'

    print('air_main setup start')
    xbee = pigpio.pi()
    xbee.set_mode(XBee_SLEEP_GPIO_NUMBER, pigpio.OUTPUT)
    xbee.write(XBee_SLEEP_GPIO_NUMBER, 0)
    print('Xbee sleep off')

    servo = pigpio.pi()
    servo.set_mode(SERVO_GPIO_NUMBER, pigpio.OUTPUT)
    dc = convert_dc_SG90(0)
    servo.hardware_PWM(SERVO_GPIO_NUMBER,FREQ,int(dc*10000))

    ms5607 = MS5607()
    with open(filename,'w') as f:
        writer = csv.writer(f)
        writer.writerow(['Temperature (degree C)','Pressure (hPa)','Altitude (m)'])

    for i in range(3):
        temperature = ms5607.getDigitalTemperature()
        pressure = ms5607.getDigitalPressure()
        default = ms5607.convertPressureTemperature(pressure, temperature)
        tmp = ms5607.getTemperature()
        time.sleep(1)

    print("Temperature: {} degrees C".format(tmp))
    print("Default Pressure(hPa): {}".format(default/100))
    print('air_main setup finish')
    print('Turn the switch.')


    input("Are you ready?")
    print('air_main start')
    xbee.write(XBee_SLEEP_GPIO_NUMBER, 1)
    time.sleep(3)
    print('Check')

    #上昇中
    while True:
        temperature = ms5607.getDigitalTemperature()
        pressure = ms5607.getDigitalPressure()
        converted = ms5607.convertPressureTemperature(pressure, temperature)
        altitude = ms5607.getMetricAltitude(converted, default)
        tmp = ms5607.getTemperature()
        print("Temperature: {} degrees C".format(tmp))
        print("Pressure(hPa): {}".format(converted/100))
        print("Altitude(m): {}".format(altitude))
        print()

        with open(filename,'a') as f:
            writer = csv.writer(f)
            writer.writerow([tmp, converted / 100, altitude])

        if altitude > ALT1:
            print("break")
            break

        time.sleep(TIME1)
        #print("up")

    while True:
        temperature = ms5607.getDigitalTemperature()
        pressure = ms5607.getDigitalPressure()
        converted = ms5607.convertPressureTemperature(pressure, temperature)
        altitude = ms5607.getMetricAltitude(converted, default)
        tmp = ms5607.getTemperature()
        print("Temperature: {} degrees C".format(tmp))
        print("Pressure(hPa): {}".format(converted/100))
        print("Altitude(m): {}".format(altitude))
        print()


        if altitude < BACKUP_ALT:

            if ONCE == True:
                time_backup = time.time()
                ONCE = False

            if altitude < ALT2 or time.time() - time_backup > BACKUP_DETACH_TIME:
                dc = convert_dc_SG90(float(ANGLE))
                servo.hardware_PWM(SERVO_GPIO_NUMBER,FREQ,int(dc*10000))
                xbee.write(XBee_SLEEP_GPIO_NUMBER, 0)

                with open(filename,'a') as f:
                    writer = csv.writer(f)
                    writer.writerow([tmp, converted / 100, altitude])

                time.sleep(3)
                if altitude < ALT2:
                    print('Detach')
                elif time.time() - time_backup > BACKUP_DETACH_TIME:
                    print('Backup detach')

                break

        with open(filename,'a') as f:
            writer = csv.writer(f)
            writer.writerow([tmp, converted / 100, altitude])

        time.sleep(TIME2)
        #print("down")

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


#if __name__ == "__main__":
#    air_main()
