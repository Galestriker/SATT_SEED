import time
from MS5607 import MS5607
import csv

sensor = MS5607()
with open('output_ms5607.csv','w') as f:
        writer = csv.writer(f)
        writer.writerow(['Temperature (degree C)','Pressure (hPa)','Altitude (m)'])


while True:
    temperature = sensor.getDigitalTemperature()
    pressure = sensor.getDigitalPressure()
    converted = sensor.convertPressureTemperature(pressure, temperature)
    altitude = sensor.getMetricAltitude(converted, 1009.7*100)#set the altimeter setting appropriately
    tmp = sensor.getTemperature()
    print("Temperature: {} degrees C".format(tmp))
    print("Pressure(hPa): {}".format(converted/100))
    print("Altitude(m): {}".format(altitude))
    print()
    
    with open('output_ms5607.csv','a') as f:
        writer = csv.writer(f)
        writer.writerow([tmp, converted/100, altitude])
    
    time.sleep(1)