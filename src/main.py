# main.py

from lib.MAX30102 import MAX30102
from utime import ticks_ms

if __name__ == '__main__':
    sensor = MAX30102()  # Loads default ESP32 I2C configuration
    # sensor = ParticleSensor(i2cHexAddress = 0x57)
    # sensor = ParticleSensor(i2cHexAddress = 0x57, i2c = i2cInstance)

    print("Setting up sensor.", '\n')
    # Setup the sensor with default configuration
    sensor.setup_sensor()
    
    print("Reading temperature in °C.", '\n')
    print(sensor.readTemperature())
    
    print("Start data acquisition from RED & IR registers.", '\n')
    red_array = list()
    ir_array = list()
    
    t_start = ticks_ms()
    samples_n=0
    while (True):
        for FIFO_pointer in range(32):
            sensor_data = sensor.read_sensor_multiLED(FIFO_pointer)
            red_array.append(sensor_data[0])
            ir_array.append(sensor_data[1])
            # print(sensor_data)
            samples_n=samples_n+1
            if (ticks_ms()-t_start) > 999:
                f_HZ = samples_n/1
                samples_n = 0
                t_start = ticks_ms()
                print("frequency = ",f_HZ)
        
    