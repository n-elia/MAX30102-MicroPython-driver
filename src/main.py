# main.py

from lib.MAX30102 import MAX30102
from utime import ticks_ms
import logging

if __name__ == '__main__':
    logging.basicConfig(level=logging.DEBUG)
    
    sensor = MAX30102()  # Loads default ESP32 I2C configuration
    # sensor = ParticleSensor(i2cHexAddress = 0x57)
    # sensor = ParticleSensor(i2cHexAddress = 0x57, i2c = i2cInstance)
    
    # Set the sample rate to 800: 800 samples/s are collected by the sensor
    sensor.setSampleRate(800)
    # Set the number of samples to be averaged per each reading
    sensor.setFIFOAverage(8)

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
    
    # Select the mode that you want to try:
    select_mode = 1
    compute_frequency = False
    
    if (select_mode == 1):
        while (True):
            # Poll continuously the sensor
            readings = sensor.burst_read()
            print(readings[0], ",", readings[1])
    
    elif (select_mode == 2):
        while (True):
            sensor.clearFIFO()
            for FIFO_pointer in range(32):
                # Acquire data from the sensor at a given FIFO position
                sensor_data = sensor.read_FIFO_on_position(FIFO_pointer)
                if sensor_data is not None:
                    # Print the acquired data
                    print(sensor_data[0], ",", sensor_data[1])
                    # Compute the real acquisition frequency
                    if (compute_frequency):
                        samples_n=samples_n+1
                        if (ticks_ms()-t_start) > 999:
                            f_HZ = samples_n/1
                            samples_n = 0
                            t_start = ticks_ms()
                            print("acquisition frequency = ",f_HZ)