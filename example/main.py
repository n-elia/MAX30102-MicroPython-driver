# main.py
from machine import sleep, SoftI2C, Pin
from utime import ticks_diff, ticks_us

from max30102 import MAX30102

if __name__ == '__main__':
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

    # Sensor instance
    sensor = MAX30102(i2c=i2c)  # An I2C instance is required

    # The default sensor configuration is:
    # Led mode: 2 (RED + IR)
    # ADC range: 16384
    # Sample rate: 400 Hz
    # Led power: maximum (50.0mA - Presence detection of ~12 inch)
    # Averaged samples: 8
    # pulse width: 411

    # It's possible to set up the sensor at once with the setup_sensor() method.
    # If no parameters are supplied, the default config is loaded.
    print("Setting up sensor with default configuration.", '\n')
    sensor.setup_sensor()

    # It is also possible to tune the configuration parameters one by one.
    # Set the sample rate to 800: 800 samples/s are collected by the sensor
    sensor.set_sample_rate(800)
    # Set the number of samples to be averaged per each reading
    sensor.set_fifo_average(8)

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