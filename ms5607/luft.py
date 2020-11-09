import time
from MS5607 import MS5607
import csv

ms5607 = MS5607()

with open('output_ms5607.csv','w') as f:
        writer = csv.writer(f)
        writer.writerow(['Temperature (degree C)','Pressure (kPa)','Altitude (m)','Difference (m)'])

temperature = ms5607.getDigitalTemperature()
pressure = ms5607.getDigitalPressure()
converted = ms5607.convertPressureTemperature(pressure, temperature)
altitude = ms5607.getMetricAltitude(converted, 1009.7*100)

default = altitude
print("Difference(m): {}".format(default))
time.sleep(3)

while True :
    temperature = ms5607.getDigitalTemperature()
    pressure = ms5607.getDigitalPressure()
    converted = ms5607.convertPressureTemperature(pressure, temperature)
    altitude = ms5607.getMetricAltitude(converted, 1009.7*100)
    tmp = ms5607.getTemperature()

    while -50000 < altitude < 50000:
        time.sleep(0.05)
        temperature = ms5607.getDigitalTemperature()
        pressure = ms5607.getDigitalPressure()
        converted = ms5607.convertPressureTemperature(pressure, temperature)
        altitude = ms5607.getMetricAltitude(converted, 1009.7*100)
        
    time.sleep(0.03)
    print("Altitude(m): {}".format(altitude))
    print("Temperature: {} degrees C".format(tmp))
    print("Pressure(kPa): {}".format(converted/1000))
    
    difference = altitude - default
    print("Difference(m): {}".format(difference))
    print(" ************************************* ")
    time.sleep(0.03)

    if difference > 15:
        break


while True :
    temperature = ms5607.getDigitalTemperature()
    pressure = ms5607.getDigitalPressure()
    converted = ms5607.convertPressureTemperature(pressure, temperature)
    altitude = ms5607.getMetricAltitude(converted, 1009.7*100)
    tmp = ms5607.getTemperature()

    while -50000 < altitude < 50000:
        time.sleep(0.03)
        temperature = ms5607.getDigitalTemperature()
        pressure = ms5607.getDigitalPressure()
        converted = ms5607.convertPressureTemperature(pressure, temperature)
        altitude = ms5607.getMetricAltitude(converted, 1009.7*100)

    print("Altitude(m): {}".format(altitude))
    print("Temperature: {} degrees C".format(tmp))
    print("Pressure(kPa): {}".format(converted/1000))

    difference = altitude - default
    print("Difference(m): {}".format(difference))
    print(" ************************************* ")
    time.sleep(0.03)

    if difference < 10:
        dc = convert_dc(float(45)) #サーボ？
        p.ChangeDutyCycle(dc)
        time.sleep(3)
        break

    with open('output_ms5607.csv','a') as f:
        writer = csv.writer(f)
        writer.writerow([tmp, converted/1000, altitude, difference])

# finishing servo
p.stop()
GPIO.cleanup()

print("Finishing")
