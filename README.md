# MAX30102 MicroPython driver

A MAX30102 driver ported to MicroPython

## Aknowledgements

This work is a lot based on:

- [SparkFun MAX3010x Sensor Library](https://github.com/sparkfun/SparkFun_MAX3010x_Sensor_Library "GitHub | SparkFun MAX3010x Sensor Library")

  Written by **Peter Jansen and Nathan Seidle** (SparkFun)
  This is a library written for the Maxim MAX30105 Optical Smoke Detector
  It should also work with the MAX30102. However, the MAX30102 does not have a Green LED.
  These sensors use I2C to communicate, as well as a single (optional)
  interrupt line that is not currently supported in this driver.
  Written by Peter Jansen and Nathan Seidle (SparkFun)
  BSD license, all text above must be included in any redistribution.

- [esp32-micropython](https://github.com/kandizzy/esp32-micropython/blob/master/PPG/ppg/MAX30105.py "GitHub | esp32-micropython")

  A port of the library to MicroPython by **kandizzy**

## Disclaimer

This work is not intended to be used in professional environments, and there are no guarantees on its functionalities. Please do not rely on it for medical purposes or professional usage.

## Repository organisation

Driver: src/lib/MAX30102.py
Example: src/main.py

## Additional information

This driver works for sure with Maxim Integrated MAX30102 sensor.
However, it *should* work with MAX30105 sensor, too.

### Usage

#### Including this library into your project

To include the library into a MicroPython project, it's sufficient to copy and include the *MAX30102.py* file. 

Example: create a *lib* directory inside your source code folder, and copy the */src/lib/MAX30102.py* file inside it.
Then, it will be possible to include the sensor instance constructor by issuing:

```python
from lib.MAX30102 import MAX30102
```

#### Setup and configuration

At first, create a sensor instance. If you leave the arguments empty, the library will load the default TinyPico ESP32 board I2C configuration (SDA Pin 21, SCL Pin 22, 400kHz speed).

```python
# Default config (ESP32):
sensor = MAX30102()

# Alternative:
from machine import SoftI2C, Pin
i2cInstance = SoftI2C(sda=Pin(my_SDA_pin),
                      scl=Pin(my_SCL_pin),
                      freq=100000)
sensor = MAX30102(i2cHexAddress = 0x57, i2c = i2cInstance)
```

Then, the sensor has to be setup. The library provides a method to setup the sensor at once. Leaving the arguments empty, makes the library load their default values.

> Default configuration values:
> 
> Led mode: 2 (RED + IR)
> 
> ADC range: 16384
> 
> Sample rate: 400 Hz
> 
> Led power: maximum (50.0mA - Presence detection of ~12 inch)
> 
> Averaged samples: 8
> 
> pulse width: 411

```python
# Setup with default values
sensor.setup_sensor()

# Alternative example:
setup_sensor(self, LED_MODE=2, ADC_RANGE=16384, SAMPLE_RATE=400)
```

The library provides the methods to change the configuration parameters one by one, too. Remember that the `setup_sensor()` method has still to be called before modifying the single parameters.

```python
# Set the number of samples to be averaged by the chip
SAMPLE_AVG = 8 # Options: 1, 2, 4, 8, 16, 32
self.setFIFOAverage(SAMPLE_AVG)

# Set the ADC range
ADC_RANGE = 4096 # Options: 2048, 4096, 8192, 16384
self.setADCRange(ADC_RANGE)

# Set the sample rate
SAMPLE_RATE = 400 # Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
self.setSampleRate(SAMPLE_RATE)

# Set the Pulse Width
PULSE_WIDTH = 118 # Options: 69, 118, 215, 411
self.setPulseWidth(PULSE_WIDTH)

# Set the LED mode
LED_MODE = 2 # Options: 1 (red), 2 (red + IR), 3 (red + IR + g - MAX30105 only)
self.setLEDMode(LED_MODE) 

# Set the LED brightness of each LED
LED_POWER = MAX30105_PULSEAMP_MEDIUM
# Options:
# MAX30105_PULSEAMP_LOWEST =  0x02 # 0.4mA  - Presence detection of ~4 inch
# MAX30105_PULSEAMP_LOW =     0x1F # 6.4mA  - Presence detection of ~8 inch
# MAX30105_PULSEAMP_MEDIUM =  0x7F # 25.4mA - Presence detection of ~8 inch
# MAX30105_PULSEAMP_HIGH =    0xFF # 50.0mA - Presence detection of ~12 inch
self.setPulseAmplitudeRed(LED_POWER)
self.setPulseAmplitudeIR(LED_POWER)
self.setPulseAmplitudeGreen(LED_POWER)

# Set the LED brightness of all the active LEDs
LED_POWER = MAX30105_PULSEAMP_MEDIUM
# Options:
# MAX30105_PULSEAMP_LOWEST =  0x02 # 0.4mA  - Presence detection of ~4 inch
# MAX30105_PULSEAMP_LOW =     0x1F # 6.4mA  - Presence detection of ~8 inch
# MAX30105_PULSEAMP_MEDIUM =  0x7F # 25.4mA - Presence detection of ~8 inch
# MAX30105_PULSEAMP_HIGH =    0xFF # 50.0mA - Presence detection of ~12 inch
sensor.setActiveLEDsAmplitude(LED_POWER)
```

#### Data acquisition

The sensor will store all the readings into a FIFO register (FIFO_DATA). Based on the number of active LEDs and other configuration paramenters, the sensor instance will read data from that register, putting it into the_storage_. The_storage_ is a circular buffer, that can be read using the provided methods.

The `check()` method polls the sensor to check if new samples are available in the FIFO queue. If data is available, it will be read and put into the _storage_. We can access those samples using the provided methods such as `popRedFromStorage()`.

As a consequence, this is an example on how the library can be used to read data from the sensor:

```python
while(True):
    # The check() method has to be continuously polled, to check if
    # there are new readings into the sensor's FIFO queue. When new
    # readings are available, this function will put them into the storage.
    sensor.check()
    
    # Check if the storage contains available samples
    if(sensor.available()):
        # Access the storage FIFO and gather the readings (integers)
        red_sample = sensor.popRedFromStorage()
        ir_sample = sensor.popIRFromStorage()
        
        # Print the acquired data (can be plot with Arduino Serial Plotter)
        print(red_sample, ",", ir_sample)
```

#### Data acquisition rate

Considering the sensor configuration, two main parameters will affect the data throughput of the sensor itself:

- The *sample rate*, which is the number of RAW readings per second made by the sensor

- The *averaged samples*, which is the number of RAW readings averaged together for composing a single sample

Therefore, the FIFO_DATA register will contain averaged RAW readings. The rate at which that register is fed depends on the two parameters: *real rate = sample rate / averaged samples*.

The library computes this value, that can be accessed with:

```python
# Get the estimated acquisition rate
acquisition_rate = sensor.getAcquisitionFrequency()
```

However, there are some limitations (on sensor side or on micropocessor side) that may affect the acquisition rate. Is is possible to "measure" it as explained in [this](https://github.com/sparkfun/SparkFun_MAX3010x_Sensor_Library/blob/master/examples/Example9_RateTesting/Example9_RateTesting.ino) example sketch by SparkFun.
Using our library, we can compute the real acquisition rate as follows.

```python
# (Assuming that the sensor instance has been already set-up)
from utime import ticks_diff, ticks_ms

# Starting time of the acquisition
t_start = ticks_ms()
# Number of samples that has been collected
samples_n = 0    

while(True):
    sensor.check()
    
    if(sensor.available()):
        # Access the storage FIFO and gather the readings (integers)
        red_sample = sensor.popRedFromStorage()
        ir_sample = sensor.popIRFromStorage()
        
        # We can compute the real frequency at which we receive data
        if (compute_frequency):
            samples_n=samples_n+1
            if ( ticks_diff(ticks_ms(), t_start) > 999 ):
                f_HZ = samples_n/1
                samples_n = 0
                t_start = ticks_ms()
                print("Acquisition frequency = ",f_HZ)
```

#### Die temperature reading

The `readTemperature()` method allows to read the internal die temperature. An example is proposed below.

```python
# Read the die temperature in Celsius degree
temperature_C = sensor.readTemperature()
print("Die temperature: ", temperature_C, "°C")
```

Note: as stated in the [datasheet](https://datasheets.maximintegrated.com/en/ds/MAX30102.pdf), the internal die temperature sensor is intended for calibrating the temperature dependence of the SpO2 subsystem. It has an inherent resolution of 0.0625°C, but be aware that the accuracy is ±1°C.

#### Realtime plot over Serial

The example proposed in this repository ([main.py](../src/main.py)) contains a print statement in this form: `print(red_reading, ",", IR_reading)`. If you open the Arduino IDE, and connect your board to it, then you will be able to open the *serial plotter* (Ctrl+Maiusc+L) and see a real-time plot of your readings (need help? take a look [here](https://learn.sparkfun.com/tutorials/max30105-particle-and-pulse-ox-sensor-hookup-guide/all)).
For instance, this is an example of my heartbeat taken on the index finger:

![Serial Plotter picture](./img/arduino-IDE-serial-plotter-heartbeat.png "Serial Plotter picture")

### Tested platforms

The example works well on TinyPico (ESP32-D4 board) running 'tinypico-20210418-v1.15.bin' MicroPython firmware, connected to a genuine Maxim 30102 breakout board ([MAXREFDES117#](https://www.maximintegrated.com/en/design/reference-design-center/system-board/6300.html)).

### Other things that it's worth mentioning

- There is an issue involving chinese clones of the Maxim MAX30102: some of them appear to have the red and IR registers inverted (or maybe the LEDs swapped) (see [here](https://github.com/aromring/MAX30102_by_RF/issues/13)). You can easily check if your sensor is inverted by putting it in LED mode 1: only the red LED should work. If you see the IR LED (use your phone camera to check), then you have to collect IR samples as red ones and viceversa.

- The sensor has to be wired to the I2C pins of your board

- If you're looking for algorithms for extracting heartrate and SPO2 from your RAW data, take a look [here](https://github.com/aromring/MAX30102_by_RF) and [here](https://github.com/kandizzy/esp32-micropython/tree/master/PPG)
