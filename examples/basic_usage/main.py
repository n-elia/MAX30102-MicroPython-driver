""" BASIC USAGE EXAMPLE
This example shows how to use the MAX30102 sensor to collect data from the RED and IR channels.

The sensor is connected to the I2C bus, and the I2C bus is scanned to ensure that the sensor is connected.
The sensor is also checked to ensure that it is a MAX30102 or MAX30105 sensor.

The sensor is set up with the following parameters:
- Sample rate: 400 Hz
- Averaged samples: 8
- LED brightness: medium
- Pulse width: 411 µs
- Led mode: 2 (RED + IR)

The temperature is read at the beginning of the acquisition.

Then, in a loop the data is printed to the serial port, so that it can be plotted with a Serial Plotter.
Also the real acquisition frequency (i.e. the rate at which samples are collected from the sensor) is computed
and printed to the serial port. It differs from the sample rate, because the sensor processed the data and
averages the samples before putting them into the FIFO queue (by default, 8 samples are averaged).

Author: n-elia
"""

# Some ports need to import 'sleep' from 'time' module
from machine import sleep, SoftI2C, Pin
from utime import ticks_diff, ticks_us

from max30102 import MAX30102, MAX30105_PULSE_AMP_MEDIUM


def main():
    # I2C software instance
    i2c = SoftI2C(sda=Pin(22),  # Here, use your I2C SDA pin
                  scl=Pin(21),  # Here, use your I2C SCL pin
                  freq=400000)  # Fast: 400kHz, slow: 100kHz

    # Examples of working I2C configurations:
    # Board             |   SDA pin  |   SCL pin
    # ------------------------------------------
    # ESP32 D1 Mini     |   22       |   21
    # TinyPico ESP32    |   21       |   22
    # Raspberry Pi Pico |   16       |   17
    # TinyS3			|	 8		 |    9

    # Sensor instance
    sensor = MAX30102(i2c=i2c)  # An I2C instance is required

    # Scan I2C bus to ensure that the sensor is connected
    if sensor.i2c_address not in i2c.scan():
        print("Sensor not found.")
        return
    elif not (sensor.check_part_id()):
        # Check that the targeted sensor is compatible
        print("I2C device ID not corresponding to MAX30102 or MAX30105.")
        return
    else:
        print("Sensor connected and recognized.")

    # It's possible to set up the sensor at once with the setup_sensor() method.
    # If no parameters are supplied, the default config is loaded:
    # Led mode: 2 (RED + IR)
    # ADC range: 16384
    # Sample rate: 400 Hz
    # Led power: maximum (50.0mA - Presence detection of ~12 inch)
    # Averaged samples: 8
    # pulse width: 411
    print("Setting up sensor with default configuration.", '\n')
    sensor.setup_sensor()

    # It is also possible to tune the configuration parameters one by one.
    # Set the sample rate to 400: 400 samples/s are collected by the sensor
    sensor.set_sample_rate(400)
    # Set the number of samples to be averaged per each reading
    sensor.set_fifo_average(8)
    # Set LED brightness to a medium value
    sensor.set_active_leds_amplitude(MAX30105_PULSE_AMP_MEDIUM)

    sleep(1)

    # The readTemperature() method allows to extract the die temperature in °C    
    print("Reading temperature in °C.", '\n')
    print(sensor.read_temperature())

    # Select whether to compute the acquisition frequency or not
    compute_frequency = True

    print("Starting data acquisition from RED & IR registers...", '\n')
    sleep(1)

    t_start = ticks_us()  # Starting time of the acquisition
    samples_n = 0  # Number of samples that have been collected

    while True:
        # The check() method has to be continuously polled, to check if
        # there are new readings into the sensor's FIFO queue. When new
        # readings are available, this function will put them into the storage.
        sensor.check()

        # Check if the storage contains available samples
        if sensor.available():
            # Access the storage FIFO and gather the readings (integers)
            red_reading = sensor.pop_red_from_storage()
            ir_reading = sensor.pop_ir_from_storage()

            # Print the acquired data (so that it can be plotted with a Serial Plotter)
            print(red_reading, ",", ir_reading)

            # Compute the real frequency at which we receive data
            if compute_frequency:
                if ticks_diff(ticks_us(), t_start) >= 999999:
                    f_HZ = samples_n
                    samples_n = 0
                    print("acquisition frequency = ", f_HZ)
                    t_start = ticks_us()
                else:
                    samples_n = samples_n + 1


if __name__ == '__main__':
    main()
