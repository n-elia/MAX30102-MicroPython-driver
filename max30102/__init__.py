# This work is a lot based on:
# - https://github.com/sparkfun/SparkFun_MAX3010x_Sensor_Library
#   Written by Peter Jansen and Nathan Seidle (SparkFun)
#   This is a library written for the Maxim MAX30105 Optical Smoke Detector
#   It should also work with the MAX30105, which has a Green LED, too.
#   These sensors use I2C to communicate, as well as a single (optional)
#   interrupt line that is not currently supported in this driver.
#   Written by Peter Jansen and Nathan Seidle (SparkFun)
#   BSD license, all text above must be included in any redistribution.
#
# - https://github.com/kandizzy/esp32-micropython/blob/master/PPG/ppg/MAX30105.py
#   A port of the library to MicroPython by kandizzy
#
# This driver aims at giving almost full access to Maxim MAX30102 functionalities.
#                                                                          n-elia

from machine import SoftI2C
from ustruct import unpack
from utime import sleep_ms, ticks_diff, ticks_ms

from max30102.circular_buffer import CircularBuffer

# I2C address (7-bit address)
MAX3010X_I2C_ADDRESS = 0x57  # Right-shift of 0xAE, 0xAF

# Status Registers
MAX30105_INT_STAT_1 = 0x00
MAX30105_INT_STAT_2 = 0x01
MAX30105_INT_ENABLE_1 = 0x02
MAX30105_INT_ENABLE_2 = 0x03

# FIFO Registers
MAX30105_FIFO_WRITE_PTR = 0x04
MAX30105_FIFO_OVERFLOW = 0x05
MAX30105_FIFO_READ_PTR = 0x06
MAX30105_FIFO_DATA = 0x07

# Configuration Registers
MAX30105_FIFO_CONFIG = 0x08
MAX30105_MODE_CONFIG = 0x09
MAX30105_PARTICLE_CONFIG = 0x0A  # Sometimes listed as 'SPO2' in datasheet (pag.11)
MAX30105_LED1_PULSE_AMP = 0x0C  # IR
MAX30105_LED2_PULSE_AMP = 0x0D  # RED
MAX30105_LED3_PULSE_AMP = 0x0E  # GREEN (when available)
MAX30105_LED_PROX_AMP = 0x10
MAX30105_MULTI_LED_CONFIG_1 = 0x11
MAX30105_MULTI_LED_CONFIG_2 = 0x12

# Die Temperature Registers
MAX30105_DIE_TEMP_INT = 0x1F
MAX30105_DIE_TEMP_FRAC = 0x20
MAX30105_DIE_TEMP_CONFIG = 0x21

# Proximity Function Registers
MAX30105_PROX_INT_THRESH = 0x30

# Part ID Registers
MAX30105_REVISION_ID = 0xFE
MAX30105_PART_ID = 0xFF  # Should always be 0x15. Identical for MAX30102.

# MAX30105 Commands
# Interrupt configuration (datasheet pag 13, 14)
MAX30105_INT_A_FULL_MASK = ~0b10000000
MAX30105_INT_A_FULL_ENABLE = 0x80
MAX30105_INT_A_FULL_DISABLE = 0x00

MAX30105_INT_DATA_RDY_MASK = ~0b01000000
MAX30105_INT_DATA_RDY_ENABLE = 0x40
MAX30105_INT_DATA_RDY_DISABLE = 0x00

MAX30105_INT_ALC_OVF_MASK = ~0b00100000
MAX30105_INT_ALC_OVF_ENABLE = 0x20
MAX30105_INT_ALC_OVF_DISABLE = 0x00

MAX30105_INT_PROX_INT_MASK = ~0b00010000
MAX30105_INT_PROX_INT_ENABLE = 0x10
MAX30105_INT_PROX_INT_DISABLE = 0x00

MAX30105_INT_DIE_TEMP_RDY_MASK = ~0b00000010
MAX30105_INT_DIE_TEMP_RDY_ENABLE = 0x02
MAX30105_INT_DIE_TEMP_RDY_DISABLE = 0x00

# FIFO data queue configuration
MAX30105_SAMPLE_AVG_MASK = ~0b11100000
MAX30105_SAMPLE_AVG_1 = 0x00
MAX30105_SAMPLE_AVG_2 = 0x20
MAX30105_SAMPLE_AVG_4 = 0x40
MAX30105_SAMPLE_AVG_8 = 0x60
MAX30105_SAMPLE_AVG_16 = 0x80
MAX30105_SAMPLE_AVG_32 = 0xA0

MAX30105_ROLLOVER_MASK = 0xEF
MAX30105_ROLLOVER_ENABLE = 0x10
MAX30105_ROLLOVER_DISABLE = 0x00
# Mask for 'almost full' interrupt (defaults to 32 samples)
MAX30105_A_FULL_MASK = 0xF0

# Mode configuration commands (page 19)
MAX30105_SHUTDOWN_MASK = 0x7F
MAX30105_SHUTDOWN = 0x80
MAX30105_WAKEUP = 0x00
MAX30105_RESET_MASK = 0xBF
MAX30105_RESET = 0x40

MAX30105_MODE_MASK = 0xF8
MAX30105_MODE_RED_ONLY = 0x02
MAX30105_MODE_RED_IR_ONLY = 0x03
MAX30105_MODE_MULTI_LED = 0x07

# Particle sensing configuration commands (pgs 19-20)
MAX30105_ADC_RANGE_MASK = 0x9F
MAX30105_ADC_RANGE_2048 = 0x00
MAX30105_ADC_RANGE_4096 = 0x20
MAX30105_ADC_RANGE_8192 = 0x40
MAX30105_ADC_RANGE_16384 = 0x60

MAX30105_SAMPLERATE_MASK = 0xE3
MAX30105_SAMPLERATE_50 = 0x00
MAX30105_SAMPLERATE_100 = 0x04
MAX30105_SAMPLERATE_200 = 0x08
MAX30105_SAMPLERATE_400 = 0x0C
MAX30105_SAMPLERATE_800 = 0x10
MAX30105_SAMPLERATE_1000 = 0x14
MAX30105_SAMPLERATE_1600 = 0x18
MAX30105_SAMPLERATE_3200 = 0x1C

MAX30105_PULSE_WIDTH_MASK = 0xFC
MAX30105_PULSE_WIDTH_69 = 0x00
MAX30105_PULSE_WIDTH_118 = 0x01
MAX30105_PULSE_WIDTH_215 = 0x02
MAX30105_PULSE_WIDTH_411 = 0x03

# LED brightness level. It affects the distance of detection.
MAX30105_PULSE_AMP_LOWEST = 0x02  # 0.4mA  - Presence detection of ~4 inch
MAX30105_PULSE_AMP_LOW = 0x1F  # 6.4mA  - Presence detection of ~8 inch
MAX30105_PULSE_AMP_MEDIUM = 0x7F  # 25.4mA - Presence detection of ~8 inch
MAX30105_PULSE_AMP_HIGH = 0xFF  # 50.0mA - Presence detection of ~12 inch

# Multi-LED Mode configuration (datasheet pag 22)
MAX30105_SLOT1_MASK = 0xF8
MAX30105_SLOT2_MASK = 0x8F
MAX30105_SLOT3_MASK = 0xF8
MAX30105_SLOT4_MASK = 0x8F
SLOT_NONE = 0x00
SLOT_RED_LED = 0x01
SLOT_IR_LED = 0x02
SLOT_GREEN_LED = 0x03
SLOT_NONE_PILOT = 0x04
SLOT_RED_PILOT = 0x05
SLOT_IR_PILOT = 0x06
SLOT_GREEN_PILOT = 0x07

MAX_30105_EXPECTED_PART_ID = 0x15

# Size of the queued readings
STORAGE_QUEUE_SIZE = 4


# Data structure to hold the last readings
class SensorData:
    def __init__(self):
        self.red = CircularBuffer(STORAGE_QUEUE_SIZE)
        self.IR = CircularBuffer(STORAGE_QUEUE_SIZE)
        self.green = CircularBuffer(STORAGE_QUEUE_SIZE)


# Sensor class
class MAX30102(object):
    def __init__(self,
                 i2c: SoftI2C,
                 i2c_hex_address=MAX3010X_I2C_ADDRESS,
                 ):
        self.i2c_address = i2c_hex_address
        self._i2c = i2c
        self._active_leds = None
        self._pulse_width = None
        self._multi_led_read_mode = None
        # Store current config values to compute acquisition frequency
        self._sample_rate = None
        self._sample_avg = None
        self._acq_frequency = None
        self._acq_frequency_inv = None
        # Circular buffer of readings from the sensor
        self.sense = SensorData()

    # Sensor setup method
    def setup_sensor(self, led_mode=2, adc_range=16384, sample_rate=400,
                     led_power=MAX30105_PULSE_AMP_MEDIUM, sample_avg=8,
                     pulse_width=411):
        # Reset the sensor's registers from previous configurations
        self.soft_reset()

        # Set the number of samples to be averaged by the chip to 8
        self.set_fifo_average(sample_avg)

        # Allow FIFO queues to wrap/roll over
        self.enable_fifo_rollover()

        # Set the LED mode to the default value of 2 (RED + IR)
        # Note: the 3rd mode is available only with MAX30105
        self.set_led_mode(led_mode)

        # Set the ADC range to default value of 16384
        self.set_adc_range(adc_range)

        # Set the sample rate to the default value of 400
        self.set_sample_rate(sample_rate)

        # Set the Pulse Width to the default value of 411
        self.set_pulse_width(pulse_width)

        # Set the LED brightness to the default value of 'low'
        self.set_pulse_amplitude_red(led_power)
        self.set_pulse_amplitude_it(led_power)
        self.set_pulse_amplitude_green(led_power)
        self.set_pulse_amplitude_proximity(led_power)

        # Clears the FIFO
        self.clear_fifo()

    def __del__(self):
        self.shutdown()

    # Methods to read the two interrupt flags
    def get_int_1(self):
        # Load the Interrupt 1 status (configurable) from the register
        rev_id = self.i2c_read_register(MAX30105_INT_STAT_1)
        return rev_id

    def get_int_2(self):
        # Load the Interrupt 2 status (DIE_TEMP_DRY) from the register
        rev_id = self.i2c_read_register(MAX30105_INT_STAT_2)
        return rev_id

    # Methods to set up the interrupt flags
    def enable_a_full(self):
        # Enable the almost full interrupt (datasheet pag. 13)
        self.bitmask(MAX30105_INT_ENABLE_1, MAX30105_INT_A_FULL_MASK, MAX30105_INT_A_FULL_ENABLE)

    def disable_a_full(self):
        # Disable the almost full interrupt (datasheet pag. 13)
        self.bitmask(MAX30105_INT_ENABLE_1, MAX30105_INT_A_FULL_MASK, MAX30105_INT_A_FULL_DISABLE)

    def enable_data_rdy(self):
        # Enable the new FIFO data ready interrupt (datasheet pag. 13)
        self.bitmask(MAX30105_INT_ENABLE_1, MAX30105_INT_DATA_RDY_MASK, MAX30105_INT_DATA_RDY_ENABLE)

    def disable_data_rdy(self):
        # Disable the new FIFO data ready interrupt (datasheet pag. 13)
        self.bitmask(MAX30105_INT_ENABLE_1, MAX30105_INT_DATA_RDY_MASK, MAX30105_INT_DATA_RDY_DISABLE)

    def enable_alc_ovf(self):
        # Enable the ambient light limit interrupt (datasheet pag. 13)
        self.bitmask(MAX30105_INT_ENABLE_1, MAX30105_INT_ALC_OVF_MASK, MAX30105_INT_ALC_OVF_ENABLE)

    def disable_alc_ovf(self):
        # Disable the ambient light limit interrupt (datasheet pag. 13)
        self.bitmask(MAX30105_INT_ENABLE_1, MAX30105_INT_ALC_OVF_MASK, MAX30105_INT_ALC_OVF_DISABLE)

    def enable_prox_int(self):
        # Enable the proximity interrupt (datasheet pag. 13)
        self.bitmask(MAX30105_INT_ENABLE_1, MAX30105_INT_PROX_INT_MASK, MAX30105_INT_PROX_INT_ENABLE)

    def disable_prox_int(self):
        # Disable the proximity interrupt (datasheet pag. 13)
        self.bitmask(MAX30105_INT_ENABLE_1, MAX30105_INT_PROX_INT_MASK, MAX30105_INT_PROX_INT_DISABLE)

    def enable_die_temp_rdy(self):
        # Enable the die temp. conversion finish interrupt (datasheet pag. 13)
        self.bitmask(MAX30105_INT_ENABLE_2, MAX30105_INT_DIE_TEMP_RDY_MASK, MAX30105_INT_DIE_TEMP_RDY_ENABLE)

    def disable_die_temp_rdy(self):
        # Disable the die temp. conversion finish interrupt (datasheet pag. 13)
        self.bitmask(MAX30105_INT_ENABLE_2, MAX30105_INT_DIE_TEMP_RDY_MASK, MAX30105_INT_DIE_TEMP_RDY_DISABLE)

        # Configuration reset

    def soft_reset(self):
        # When the RESET bit is set to one, all configuration, threshold,
        # and data registers are reset to their power-on-state through
        # a power-on reset. The RESET bit is cleared automatically back to zero
        # after the reset sequence is completed. (datasheet pag. 19)
        self.set_bitmask(MAX30105_MODE_CONFIG, MAX30105_RESET_MASK, MAX30105_RESET)
        curr_status = -1
        while not ((curr_status & MAX30105_RESET) == 0):
            sleep_ms(10)
            curr_status = ord(self.i2c_read_register(MAX30105_MODE_CONFIG))

    # Power states methods
    def shutdown(self):
        # Put IC into low power mode (datasheet pg. 19)
        # During shutdown the IC will continue to respond to I2C commands but
        # will not update with or take new readings (such as temperature).
        self.set_bitmask(MAX30105_MODE_CONFIG, MAX30105_SHUTDOWN_MASK, MAX30105_SHUTDOWN)

    def wakeup(self):
        # Pull IC out of low power mode (datasheet pg. 19)
        self.set_bitmask(MAX30105_MODE_CONFIG, MAX30105_SHUTDOWN_MASK, MAX30105_WAKEUP)

        # LED Configuration

    def set_led_mode(self, LED_mode):
        # Set LED mode: select which LEDs are used for sampling
        # Options: RED only, RED + IR only, or ALL (datasheet pag. 19)
        if LED_mode == 1:
            self.set_bitmask(MAX30105_MODE_CONFIG, MAX30105_MODE_MASK, MAX30105_MODE_RED_ONLY)
        elif LED_mode == 2:
            self.set_bitmask(MAX30105_MODE_CONFIG, MAX30105_MODE_MASK, MAX30105_MODE_RED_IR_ONLY)
        elif LED_mode == 3:
            self.set_bitmask(MAX30105_MODE_CONFIG, MAX30105_MODE_MASK, MAX30105_MODE_MULTI_LED)
        else:
            raise ValueError('Wrong LED mode:{0}!'.format(LED_mode))

        # Multi-LED Mode Configuration: enable the reading of the LEDs
        # depending on the chosen mode
        self.enable_slot(1, SLOT_RED_LED)
        if LED_mode > 1:
            self.enable_slot(2, SLOT_IR_LED)
        if LED_mode > 2:
            self.enable_slot(3, SLOT_GREEN_LED)

        # Store the LED mode used to control how many bytes to read from
        # FIFO buffer in multiLED mode: a sample is made of 3 bytes
        self._active_leds = LED_mode
        self._multi_led_read_mode = LED_mode * 3

    # ADC Configuration
    def set_adc_range(self, ADC_range):
        # ADC range: set the range of the conversion
        # Options: 2048, 4096, 8192, 16384
        # Current draw: 7.81pA. 15.63pA, 31.25pA, 62.5pA per LSB.
        if ADC_range == 2048:
            r = MAX30105_ADC_RANGE_2048
        elif ADC_range == 4096:
            r = MAX30105_ADC_RANGE_4096
        elif ADC_range == 8192:
            r = MAX30105_ADC_RANGE_8192
        elif ADC_range == 16384:
            r = MAX30105_ADC_RANGE_16384
        else:
            raise ValueError('Wrong ADC range:{0}!'.format(ADC_range))

        self.set_bitmask(MAX30105_PARTICLE_CONFIG, MAX30105_ADC_RANGE_MASK, r)

    # Sample Rate Configuration
    def set_sample_rate(self, sample_rate):
        # Sample rate: select the number of samples taken per second.
        # Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
        # Note: in theory, the resulting acquisition frequency for the end user
        # is sampleRate/sampleAverage. However, it is worth testing it before
        # assuming that the sensor can effectively sustain that frequency
        # given its configuration.
        if sample_rate == 50:
            sr = MAX30105_SAMPLERATE_50
        elif sample_rate == 100:
            sr = MAX30105_SAMPLERATE_100
        elif sample_rate == 200:
            sr = MAX30105_SAMPLERATE_200
        elif sample_rate == 400:
            sr = MAX30105_SAMPLERATE_400
        elif sample_rate == 800:
            sr = MAX30105_SAMPLERATE_800
        elif sample_rate == 1000:
            sr = MAX30105_SAMPLERATE_1000
        elif sample_rate == 1600:
            sr = MAX30105_SAMPLERATE_1600
        elif sample_rate == 3200:
            sr = MAX30105_SAMPLERATE_3200
        else:
            raise ValueError('Wrong sample rate:{0}!'.format(sample_rate))

        self.set_bitmask(MAX30105_PARTICLE_CONFIG, MAX30105_SAMPLERATE_MASK, sr)

        # Store the sample rate and recompute the acq. freq.
        self._sample_rate = sample_rate
        self.update_acquisition_frequency()

    # Pulse width Configuration
    def set_pulse_width(self, pulse_width):
        # Pulse width of LEDs: The longer the pulse width the longer range of
        # detection. At 69us and 0.4mA it's about 2 inches,
        # at 411us and 0.4mA it's about 6 inches.
        if pulse_width == 69:
            pw = MAX30105_PULSE_WIDTH_69
        elif pulse_width == 118:
            pw = MAX30105_PULSE_WIDTH_118
        elif pulse_width == 215:
            pw = MAX30105_PULSE_WIDTH_215
        elif pulse_width == 411:
            pw = MAX30105_PULSE_WIDTH_411
        else:
            raise ValueError('Wrong pulse width:{0}!'.format(pulse_width))
        self.set_bitmask(MAX30105_PARTICLE_CONFIG, MAX30105_PULSE_WIDTH_MASK, pw)

        # Store the pulse width
        self._pulse_width = pw

    # LED Pulse Amplitude Configuration methods
    def set_active_leds_amplitude(self, amplitude):
        if self._active_leds > 0:
            self.set_pulse_amplitude_red(amplitude)
        if self._active_leds > 1:
            self.set_pulse_amplitude_it(amplitude)
        if self._active_leds > 2:
            self.set_pulse_amplitude_green(amplitude)

    def set_pulse_amplitude_red(self, amplitude):
        self.i2c_set_register(MAX30105_LED1_PULSE_AMP, amplitude)

    def set_pulse_amplitude_it(self, amplitude):
        self.i2c_set_register(MAX30105_LED2_PULSE_AMP, amplitude)

    def set_pulse_amplitude_green(self, amplitude):
        self.i2c_set_register(MAX30105_LED3_PULSE_AMP, amplitude)

    def set_pulse_amplitude_proximity(self, amplitude):
        self.i2c_set_register(MAX30105_LED_PROX_AMP, amplitude)

    def set_proximity_threshold(self, thresh_msb):
        # Set the IR ADC count that will trigger the beginning of particle-
        # sensing mode.The threshMSB signifies only the 8 most significant-bits
        # of the ADC count. (datasheet page 24)
        self.i2c_set_register(MAX30105_PROX_INT_THRESH, thresh_msb)

    # FIFO averaged samples number Configuration
    def set_fifo_average(self, number_of_samples):
        # FIFO sample avg: set the number of samples to be averaged by the chip.
        # Options: MAX30105_SAMPLE_AVG_1, 2, 4, 8, 16, 32
        if number_of_samples == 1:
            ns = MAX30105_SAMPLE_AVG_1
        elif number_of_samples == 2:
            ns = MAX30105_SAMPLE_AVG_2
        elif number_of_samples == 4:
            ns = MAX30105_SAMPLE_AVG_4
        elif number_of_samples == 8:
            ns = MAX30105_SAMPLE_AVG_8
        elif number_of_samples == 16:
            ns = MAX30105_SAMPLE_AVG_16
        elif number_of_samples == 32:
            ns = MAX30105_SAMPLE_AVG_32
        else:
            raise ValueError(
                'Wrong number of samples:{0}!'.format(number_of_samples))
        self.set_bitmask(MAX30105_FIFO_CONFIG, MAX30105_SAMPLE_AVG_MASK, ns)

        # Store the number of averaged samples and recompute the acq. freq.
        self._sample_avg = number_of_samples
        self.update_acquisition_frequency()

    def update_acquisition_frequency(self):
        if None in [self._sample_rate, self._sample_avg]:
            return
        else:
            self._acq_frequency = self._sample_rate / self._sample_avg
            from math import ceil

            # Compute the time interval to wait before taking a good measure
            # (see note in setSampleRate() method)
            self._acq_frequency_inv = int(ceil(1000 / self._acq_frequency))

    def get_acquisition_frequency(self):
        return self._acq_frequency

    def clear_fifo(self):
        # Resets all points to start in a known state
        # Datasheet page 15 recommends clearing FIFO before beginning a read
        self.i2c_set_register(MAX30105_FIFO_WRITE_PTR, 0)
        self.i2c_set_register(MAX30105_FIFO_OVERFLOW, 0)
        self.i2c_set_register(MAX30105_FIFO_READ_PTR, 0)

    def enable_fifo_rollover(self):
        # FIFO rollover: enable to allow FIFO tro wrap/roll over
        self.set_bitmask(MAX30105_FIFO_CONFIG, MAX30105_ROLLOVER_MASK, MAX30105_ROLLOVER_ENABLE)

    def disable_fifo_rollover(self):
        # FIFO rollover: disable to disallow FIFO tro wrap/roll over
        self.set_bitmask(MAX30105_FIFO_CONFIG, MAX30105_ROLLOVER_MASK, MAX30105_ROLLOVER_DISABLE)

    def set_fifo_almost_full(self, number_of_samples):
        # Set number of samples to trigger the almost full interrupt (page 18)
        # Power on default is 32 samples. Note it is reverse: 0x00 is
        # 32 samples, 0x0F is 17 samples
        self.set_bitmask(MAX30105_FIFO_CONFIG, MAX30105_A_FULL_MASK, number_of_samples)

    def get_write_pointer(self):
        # Read the FIFO Write Pointer from the register
        wp = self.i2c_read_register(MAX30105_FIFO_WRITE_PTR)
        return wp

    def get_read_pointer(self):
        # Read the FIFO Read Pointer from the register
        wp = self.i2c_read_register(MAX30105_FIFO_READ_PTR)
        return wp

    # Die Temperature method: returns the temperature in C
    def read_temperature(self):
        # DIE_TEMP_RDY interrupt must be enabled
        # Config die temperature register to take 1 temperature sample
        self.i2c_set_register(MAX30105_DIE_TEMP_CONFIG, 0x01)

        # Poll for bit to clear, reading is then complete
        reading = ord(self.i2c_read_register(MAX30105_INT_STAT_2))
        sleep_ms(100)
        while (reading & MAX30105_INT_DIE_TEMP_RDY_ENABLE) > 0:
            reading = ord(self.i2c_read_register(MAX30105_INT_STAT_2))
            sleep_ms(1)

        # Read die temperature register (integer)
        tempInt = ord(self.i2c_read_register(MAX30105_DIE_TEMP_INT))
        # Causes the clearing of the DIE_TEMP_RDY interrupt
        tempFrac = ord(self.i2c_read_register(MAX30105_DIE_TEMP_FRAC))

        # Calculate temperature (datasheet pg. 23)
        return float(tempInt) + (float(tempFrac) * 0.0625)

    def set_prox_int_tresh(self, val):
        # Set the PROX_INT_THRESH (see proximity function on datasheet, pag 10)
        self.i2c_set_register(MAX30105_PROX_INT_THRESH, val)

    # DeviceID and Revision methods
    def read_part_id(self):
        # Load the Device ID from the register
        part_id = self.i2c_read_register(MAX30105_PART_ID)
        return part_id

    def check_part_id(self):
        # Checks the correctness of the Device ID
        part_id = ord(self.read_part_id())
        return part_id == MAX_30105_EXPECTED_PART_ID

    def get_revision_id(self):
        # Load the Revision ID from the register
        rev_id = self.i2c_read_register(MAX30105_REVISION_ID)
        return ord(rev_id)

    # Time slots management for multi-LED operation mode
    def enable_slot(self, slot_number, device):
        # In multi-LED mode, each sample is split into up to four time slots,
        # SLOT1 through SLOT4. These control registers determine which LED is
        # active in each time slot. (datasheet pag 22)
        # Devices are SLOT_RED_LED or SLOT_RED_PILOT (proximity)
        # Assigning a SLOT_RED_LED will pulse LED
        # Assigning a SLOT_RED_PILOT will detect the proximity
        if slot_number == 1:
            self.bitmask(MAX30105_MULTI_LED_CONFIG_1, MAX30105_SLOT1_MASK, device)
        elif slot_number == 2:
            self.bitmask(MAX30105_MULTI_LED_CONFIG_1, MAX30105_SLOT2_MASK, device << 4)
        elif slot_number == 3:
            self.bitmask(MAX30105_MULTI_LED_CONFIG_2, MAX30105_SLOT3_MASK, device)
        elif slot_number == 4:
            self.bitmask(MAX30105_MULTI_LED_CONFIG_2, MAX30105_SLOT4_MASK, device << 4)
        else:
            raise ValueError('Wrong slot number:{0}!'.format(slot_number))

    def disable_slots(self):
        # Clear all the slots assignments
        self.i2c_set_register(MAX30105_MULTI_LED_CONFIG_1, 0)
        self.i2c_set_register(MAX30105_MULTI_LED_CONFIG_2, 0)

    # Low-level I2C Communication
    def i2c_read_register(self, REGISTER, n_bytes=1):
        self._i2c.writeto(self.i2c_address, bytearray([REGISTER]))
        return self._i2c.readfrom(self.i2c_address, n_bytes)

    def i2c_set_register(self, REGISTER, VALUE):
        self._i2c.writeto(self.i2c_address, bytearray([REGISTER, VALUE]))
        return

    # Given a register, read it, mask it, and then set the thing
    def set_bitmask(self, REGISTER, MASK, NEW_VALUES):
        newCONTENTS = (ord(self.i2c_read_register(REGISTER)) & MASK) | NEW_VALUES
        self.i2c_set_register(REGISTER, newCONTENTS)
        return

    # Given a register, read it and mask it
    def bitmask(self, reg, slotMask, thing):
        originalContents = ord(self.i2c_read_register(reg))
        originalContents = originalContents & slotMask
        self.i2c_set_register(reg, originalContents | thing)

    def fifo_bytes_to_int(self, fifo_bytes):
        value = unpack(">i", b'\x00' + fifo_bytes)
        return (value[0] & 0x3FFFF) >> self._pulse_width

    # Returns how many samples are available
    def available(self):
        number_of_samples = len(self.sense.red)
        return number_of_samples

    # Get a new red value
    def get_red(self):
        # Check the sensor for new data for 250ms
        if self.safe_check(250):
            return self.sense.red.pop_head()
        else:
            # Sensor failed to find new data
            return 0

    # Get a new IR value
    def get_ir(self):
        # Check the sensor for new data for 250ms
        if self.safe_check(250):
            return self.sense.IR.pop_head()
        else:
            # Sensor failed to find new data
            return 0

    # Get a new green value
    def get_green(self):
        # Check the sensor for new data for 250ms
        if self.safe_check(250):
            return self.sense.green.pop_head()
        else:
            # Sensor failed to find new data
            return 0

    # Note: the following 3 functions are the equivalent of using 'getFIFO'
    # methods of the SparkFun library
    # Pops the next red value in storage (if available)
    def pop_red_from_storage(self):
        if len(self.sense.red) == 0:
            return 0
        else:
            return self.sense.red.pop()

    # Pops the next IR value in storage (if available)
    def pop_ir_from_storage(self):
        if len(self.sense.IR) == 0:
            return 0
        else:
            return self.sense.IR.pop()

    # Pops the next green value in storage (if available)
    def pop_green_from_storage(self):
        if len(self.sense.green) == 0:
            return 0
        else:
            return self.sense.green.pop()

    # (useless - for comparison purposes only)
    def next_sample(self):
        if self.available():
            # With respect to the SparkFun library, using a deque object
            # allows us to avoid manually advancing of the tail
            return True

    # Polls the sensor for new data
    def check(self):
        # Call continuously to poll the sensor for new data.
        read_pointer = ord(self.get_read_pointer())
        write_pointer = ord(self.get_write_pointer())

        # Do we have new data?
        if read_pointer != write_pointer:
            # Calculate the number of readings we need to get from sensor
            number_of_samples = write_pointer - read_pointer

            # Wrap condition (return to the beginning of 32 samples)
            if number_of_samples < 0:
                number_of_samples += 32

            for i in range(number_of_samples):
                # Read a number of bytes equal to activeLEDs*3 (= 1 sample)
                fifo_bytes = self.i2c_read_register(MAX30105_FIFO_DATA,
                                                    self._multi_led_read_mode)

                # Convert the readings from bytes to integers, depending
                # on the number of active LEDs
                if self._active_leds > 0:
                    self.sense.red.append(
                        self.fifo_bytes_to_int(fifo_bytes[0:3])
                    )

                if self._active_leds > 1:
                    self.sense.IR.append(
                        self.fifo_bytes_to_int(fifo_bytes[3:6])
                    )

                if self._active_leds > 2:
                    self.sense.green.append(
                        self.fifo_bytes_to_int(fifo_bytes[6:9])
                    )

                return True

        else:
            return False

    # Check for new data but give up after a certain amount of time
    def safe_check(self, max_time_to_check):
        mark_time = ticks_ms()
        while True:
            if ticks_diff(ticks_ms(), mark_time) > max_time_to_check:
                # Timeout reached
                return False
            if self.check():
                # new data found
                return True
            sleep_ms(1)
