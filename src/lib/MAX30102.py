'''
This work is a lot based on:
- https://github.com/sparkfun/SparkFun_MAX3010x_Sensor_Library
  Written by Peter Jansen and Nathan Seidle (SparkFun)
  This is a library written for the Maxim MAX30105 Optical Smoke Detector
  It should also work with the MAX30102. However, the MAX30102 does not have a Green LED.
  These sensors use I2C to communicate, as well as a single (optional)
  interrupt line that is not currently supported in this driver.
  Written by Peter Jansen and Nathan Seidle (SparkFun)
  BSD license, all text above must be included in any redistribution.

- https://github.com/kandizzy/esp32-micropython/blob/master/PPG/ppg/MAX30105.py
  A port of the library to MicroPython by kandizzy

With this driver, I want to give an almost full access to Maxim MAX30102 sensor 
functionalities.
This code is being tested on TinyPico Board with Maxim original sensors.
                                                                         n-elia
'''

from ustruct import unpack
from machine import SoftI2C, Pin
from utime import sleep_ms, ticks_ms, ticks_diff
from ucollections import deque

import logging

# Setup of logger
logger = logging.getLogger("MAX30102")

# These I2C default settings work for TinyPico (ESP32-based board)
MAX3010X_I2C_ADDRESS = 0x57
I2C_SPEED_FAST =   400000  # 400kHz speed
I2C_SPEED_NORMAL = 100000  # 100kHz speed
I2C_DEF_SDA_PIN = 21
I2C_DEF_SCL_PIN = 22

# Status Registers
MAX30105_INTSTAT1 =     0x00
MAX30105_INTSTAT2 =     0x01
MAX30105_INTENABLE1 =   0x02
MAX30105_INTENABLE2 =   0x03

# FIFO Registers
MAX30105_FIFOWRITEPTR = 0x04
MAX30105_FIFOOVERFLOW = 0x05
MAX30105_FIFOREADPTR = 	0x06
MAX30105_FIFODATA =		0x07

# Configuration Registers
MAX30105_FIFOCONFIG =       0x08
MAX30105_MODECONFIG =       0x09
MAX30105_PARTICLECONFIG =   0x0A # Sometimes listed as 'SPO2' in datasheet (pag.11)
MAX30105_LED1_PULSEAMP =    0x0C # IR
MAX30105_LED2_PULSEAMP =    0x0D # RED
MAX30105_LED3_PULSEAMP =    0x0E # GREEN (when available)
MAX30105_LED_PROX_AMP =     0x10
MAX30105_MULTILEDCONFIG1 =  0x11
MAX30105_MULTILEDCONFIG2 =  0x12

# Die Temperature Registers
MAX30105_DIETEMPINT =    0x1F
MAX30105_DIETEMPFRAC =   0x20
MAX30105_DIETEMPCONFIG = 0x21

# Proximity Function Registers
MAX30105_PROXINTTHRESH = 0x30

# Part ID Registers
MAX30105_REVISIONID = 0xFE
MAX30105_PARTID =     0xFF # Should always be 0x15. Identical for MAX30102.

# MAX30105 Commands
# Interrupt configuration (datasheet pag 13, 14)
MAX30105_INT_A_FULL_MASK =      ~0b10000000
MAX30105_INT_A_FULL_ENABLE =    0x80
MAX30105_INT_A_FULL_DISABLE =   0x00

MAX30105_INT_DATA_RDY_MASK =    ~0b01000000
MAX30105_INT_DATA_RDY_ENABLE =  0x40
MAX30105_INT_DATA_RDY_DISABLE = 0x00

MAX30105_INT_ALC_OVF_MASK =     ~0b00100000
MAX30105_INT_ALC_OVF_ENABLE =   0x20
MAX30105_INT_ALC_OVF_DISABLE =  0x00

MAX30105_INT_PROX_INT_MASK =    ~0b00010000
MAX30105_INT_PROX_INT_ENABLE =  0x10
MAX30105_INT_PROX_INT_DISABLE = 0x00

MAX30105_INT_DIE_TEMP_RDY_MASK =    ~0b00000010
MAX30105_INT_DIE_TEMP_RDY_ENABLE =  0x02
MAX30105_INT_DIE_TEMP_RDY_DISABLE = 0x00

# FIFO data queue configuration
MAX30105_SAMPLEAVG_MASK =   ~0b11100000
MAX30105_SAMPLEAVG_1 =      0x00
MAX30105_SAMPLEAVG_2 = 	    0x20
MAX30105_SAMPLEAVG_4 = 	    0x40
MAX30105_SAMPLEAVG_8 = 	    0x60
MAX30105_SAMPLEAVG_16 =     0x80
MAX30105_SAMPLEAVG_32 =     0xA0

MAX30105_ROLLOVER_MASK =    0xEF
MAX30105_ROLLOVER_ENABLE =  0x10
MAX30105_ROLLOVER_DISABLE = 0x00
# Mask for 'almost full' interrupt (defaults to 32 samples)
MAX30105_A_FULL_MASK =      0xF0

# Mode configuration commands (page 19)
MAX30105_SHUTDOWN_MASK =    0x7F
MAX30105_SHUTDOWN = 	    0x80
MAX30105_WAKEUP = 		    0x00
MAX30105_RESET_MASK = 	    0xBF
MAX30105_RESET = 		    0x40

MAX30105_MODE_MASK = 	    0xF8
MAX30105_MODE_REDONLY =     0x02
MAX30105_MODE_REDIRONLY =   0x03
MAX30105_MODE_MULTILED =    0x07

# Particle sensing configuration commands (pgs 19-20)
MAX30105_ADCRANGE_MASK = 	0x9F
MAX30105_ADCRANGE_2048 = 	0x00
MAX30105_ADCRANGE_4096 = 	0x20
MAX30105_ADCRANGE_8192 = 	0x40
MAX30105_ADCRANGE_16384 = 	0x60

MAX30105_SAMPLERATE_MASK =  0xE3
MAX30105_SAMPLERATE_50 = 	0x00
MAX30105_SAMPLERATE_100 = 	0x04
MAX30105_SAMPLERATE_200 = 	0x08
MAX30105_SAMPLERATE_400 = 	0x0C
MAX30105_SAMPLERATE_800 = 	0x10
MAX30105_SAMPLERATE_1000 =  0x14
MAX30105_SAMPLERATE_1600 =  0x18
MAX30105_SAMPLERATE_3200 =  0x1C

MAX30105_PULSEWIDTH_MASK =  0xFC
MAX30105_PULSEWIDTH_69 = 	0x00
MAX30105_PULSEWIDTH_118 = 	0x01
MAX30105_PULSEWIDTH_215 = 	0x02
MAX30105_PULSEWIDTH_411 = 	0x03

# LED brigthness level. It affects the distance of detection.
MAX30105_PULSEAMP_LOWEST =  0x02 # 0.4mA  - Presence detection of ~4 inch
MAX30105_PULSEAMP_LOW =     0x1F # 6.4mA  - Presence detection of ~8 inch
MAX30105_PULSEAMP_MEDIUM =  0x7F # 25.4mA - Presence detection of ~8 inch
MAX30105_PULSEAMP_HIGH =    0xFF # 50.0mA - Presence detection of ~12 inch

# Multi-LED Mode configuration (datasheet pag 22)
MAX30105_SLOT1_MASK =   0xF8
MAX30105_SLOT2_MASK =   0x8F
MAX30105_SLOT3_MASK =   0xF8
MAX30105_SLOT4_MASK =   0x8F
SLOT_NONE =         0x00
SLOT_RED_LED =      0x01
SLOT_IR_LED =       0x02
SLOT_GREEN_LED =    0x03
SLOT_NONE_PILOT =   0x04
SLOT_RED_PILOT =    0x05
SLOT_IR_PILOT =     0x06
SLOT_GREEN_PILOT =  0x07

MAX_30105_EXPECTEDPARTID = 0x15;

TAG = 'MAX30105'

# Size of the queued readings
STORAGE_QUEUE_SIZE = 4

# Very rough implementation of a circular buffer based on deque
class CircularBuffer(object):
    def __init__(self, maxSize):
        self.data = deque((), maxSize, True)
        self.maxSize = maxSize
        
    def __len__(self):
        return len(self.data)
    
    def isEmpty(self):
        return not bool(self.data)
    
    def append(self, item):
        try:
            self.data.append(item)
        except IndexError:
            # deque full, popping 1st item out
            self.data.popleft()
            self.data.append(item)
        
    def pop(self):
        return self.data.popleft()

    def clear(self):
        self.data = deque((), self.maxSize, True)
    
    def popHead(self):
        bufferSize = len(self.data)
        temp = deque((), self.maxSize, True)
        temp = self.data
        if (bufferSize == 1):
            pass
        elif (bufferSize > 1):
            self.data.clear()
            for x in range(bufferSize - 1):
                self.data = temp.popleft()
        else:
            return 0
        return temp.popleft()
    
# Data structure to hold the last readings
class SensorData():
    def __init__(self):
        self.red = CircularBuffer(STORAGE_QUEUE_SIZE)
        self.IR  = CircularBuffer(STORAGE_QUEUE_SIZE)
        self.green = CircularBuffer(STORAGE_QUEUE_SIZE)

# Sensor class
class MAX30102(object):
    def __init__(self,
                 i2cHexAddress=MAX3010X_I2C_ADDRESS,
                 i2c=SoftI2C(sda=Pin(I2C_DEF_SDA_PIN),
                             scl=Pin(I2C_DEF_SCL_PIN),
                             freq=I2C_SPEED_FAST)
                 ):
        self._address = i2cHexAddress
        self._i2c = i2c
        self._activeLEDs = None
        self._pulseWidth = None
        self._multiLedReadMode = None
        # Store current config values to compute acquisition frequency
        self._sampleRate = None
        self._sampleAvg  = None
        self._acqFrequency = None
        self._acqFrequencyinv = None
        # Circular buffer of readings from the sensor
        self.sense = SensorData()
        
        try:
            self._i2c.readfrom(self._address, 1)
        except OSError as error:
            logger.error("(%s) I2C Error. Unable to find a MAX3010x sensor.", TAG)
            raise SystemExit(error)
        else:
            logger.info("(%s) MAX3010x sensor found!", TAG)
            
        if not (self.checkPartID()):
            logger.error("(%s) Wrong PartID. Unable to find a MAX3010x sensor.", TAG)
            raise SystemExit()
    
    # Sensor setup method
    def setup_sensor(self, LED_MODE=2, ADC_RANGE=16384, SAMPLE_RATE=400,
                     LED_POWER=MAX30105_PULSEAMP_HIGH, SAMPLE_AVG=8,
                     PULSE_WIDTH=411):
        # Reset the sensor's registers from previous configurations
        self.softReset()
        
        # Set the number of samples to be averaged by the chip to 8
        self.setFIFOAverage(SAMPLE_AVG)
        
        # Allow FIFO queue to wrap/roll over
        self.enableFIFORollover()
        
        # Set the LED mode to the default value of 2 (RED + IR)
        # Note: the 3rd mode is available only with MAX30105
        self.setLEDMode(LED_MODE)    
            
        # Set the ADC range to default value of 16384
        self.setADCRange(ADC_RANGE)
        
         # Set the sample rate to the default value of 400
        self.setSampleRate(SAMPLE_RATE)
        
        # Set the Pulse Width to the default value of 411
        self.setPulseWidth(PULSE_WIDTH)
        
        # Set the LED brightness to the default value of 'low'
        self.setPulseAmplitudeRed(LED_POWER)
        self.setPulseAmplitudeIR(LED_POWER)
        self.setPulseAmplitudeGreen(LED_POWER)
        self.setPulseAmplitudeProximity(LED_POWER)

        # Clears the FIFO
        self.clearFIFO()
     
    def __del__(self):
        self.shutDown()
        logger.info("(%s) Shutting down the sensor.", TAG)
    
    # Methods to read the two interrupt flags
    def getINT1(self):
        # Load the Interrupt 1 status (configurable) from the register
        rev_id = self.i2c_read_register(MAX30105_INTSTAT1)
        return rev_id
    
    def getINT2(self):
        # Load the Interrupt 2 status (DIE_TEMP_DRY) from the register
        rev_id = self.i2c_read_register(MAX30105_INTSTAT2)
        return rev_id
    
    # Methods to setup the interrupt flags
    def enableAFULL(self):
        # Enable the almost full interrupt (datasheet pag. 13)
        self.bitMask(MAX30105_INTENABLE1,
                     MAX30105_INT_A_FULL_MASK,
                     MAX30105_INT_A_FULL_ENABLE)
        
    def disableAFULL(self):
        # Disable the almost full interrupt (datasheet pag. 13)
        self.bitMask(MAX30105_INTENABLE1,
                     MAX30105_INT_A_FULL_MASK,
                     MAX30105_INT_A_FULL_DISABLE)
        
    def enableDATARDY(self):
        # Enable the new FIFO data ready interrupt (datasheet pag. 13)
        self.bitMask(MAX30105_INTENABLE1,
                     MAX30105_INT_DATA_RDY_MASK,
                     MAX30105_INT_DATA_RDY_ENABLE)
        
    def disableDATARDY(self):
        # Disable the new FIFO data ready interrupt (datasheet pag. 13)
        self.bitMask(MAX30105_INTENABLE1,
                     MAX30105_INT_DATA_RDY_MASK,
                     MAX30105_INT_DATA_RDY_DISABLE)
        
    def enableALCOVF(self):
        # Enable the ambient light limit interrupt (datasheet pag. 13)
        self.bitMask(MAX30105_INTENABLE1,
                     MAX30105_INT_ALC_OVF_MASK,
                     MAX30105_INT_ALC_OVF_ENABLE)
        
    def disableALCOVF(self):
        # Disable the ambient light limit interrupt (datasheet pag. 13)
        self.bitMask(MAX30105_INTENABLE1,
                     MAX30105_INT_ALC_OVF_MASK,
                     MAX30105_INT_ALC_OVF_DISABLE)  
          
    def enablePROXINT(self):
        # Enable the proximity interrupt (datasheet pag. 13)
        self.bitMask(MAX30105_INTENABLE1,
                     MAX30105_INT_PROX_INT_MASK,
                     MAX30105_INT_PROX_INT_ENABLE)
        
    def disablePROXINT(self):
        # Disable the proximity interrupt (datasheet pag. 13)
        self.bitMask(MAX30105_INTENABLE1,
                     MAX30105_INT_PROX_INT_MASK,
                     MAX30105_INT_PROX_INT_DISABLE)
        
    def enableDIETEMPRDY(self):
        # Enable the die temp. conversion finish interrupt (datasheet pag. 13)
        self.bitMask(MAX30105_INTENABLE2,
                     MAX30105_INT_DIE_TEMP_RDY_MASK,
                     MAX30105_INT_DIE_TEMP_RDY_ENABLE)
        
    def disableDIETEMPRDY(self):
        # Disable the die temp. conversion finish interrupt (datasheet pag. 13)
        self.bitMask(MAX30105_INTENABLE2,
                     MAX30105_INT_DIE_TEMP_RDY_MASK,
                     MAX30105_INT_DIE_TEMP_RDY_DISABLE)     
    
    # Configuration reset
    def softReset(self):
        TAG = 'softReset'
        # When the RESET bit is set to one, all configuration, threshold,
        # and data registers are reset to their power-on-state through
        # a power-on reset. The RESET bit is cleared automatically back to zero
        # after the reset sequence is completed. (datasheet pag. 19)
        logger.debug("(%s) Resetting the sensor.", TAG)
        self.set_bitMask(MAX30105_MODECONFIG,
                         MAX30105_RESET_MASK,
                         MAX30105_RESET)
        curr_status = -1
        while not ( (curr_status & MAX30105_RESET) == 0 ):
            sleep_ms(10)
            curr_status = ord(self.i2c_read_register(MAX30105_MODECONFIG))
    
    # Power states methods
    def shutDown(self):
        # Put IC into low power mode (datasheet pg. 19)
        # During shutdown the IC will continue to respond to I2C commands but 
        # will not update with or take new readings (such as temperature).
        self.set_bitMask(MAX30105_MODECONFIG,
                         MAX30105_SHUTDOWN_MASK,
                         MAX30105_SHUTDOWN)
        
    def wakeUp(self):
        # Pull IC out of low power mode (datasheet pg. 19)
        self.set_bitMask(MAX30105_MODECONFIG,
                         MAX30105_SHUTDOWN_MASK,
                         MAX30105_WAKEUP)   
    
    # LED Configuration
    def setLEDMode(self, LED_mode):
        # Set LED mode: select which LEDs are used for sampling 
        # Options: RED only, RED + IR only, or ALL (datasheet pag. 19)        
        if LED_mode == 1:
            self.set_bitMask(MAX30105_MODECONFIG,
                             MAX30105_MODE_MASK,
                             MAX30105_MODE_REDONLY)
        elif LED_mode == 2:
            self.set_bitMask(MAX30105_MODECONFIG,
                             MAX30105_MODE_MASK,
                             MAX30105_MODE_REDIRONLY)
        elif LED_mode == 3:
            self.set_bitMask(MAX30105_MODECONFIG,
                             MAX30105_MODE_MASK,
                             MAX30105_MODE_MULTILED)
        else:
            raise ValueError('Wrong LED mode:{0}!'.format(LED_mode))
        
        # Multi-LED Mode Configuration: enable the reading of the LEDs
        # depending on the chosen mode
        self.enableSlot(1, SLOT_RED_LED)
        if (LED_mode > 1):
            self.enableSlot(2, SLOT_IR_LED)
        if (LED_mode > 2):
            self.enableSlot(3, SLOT_GREEN_LED)
            
        # Store the LED mode used to control how many bytes to read from
        # FIFO buffer in multiLED mode: a sample is made of 3 bytes
        self._activeLEDs = LED_mode
        self._multiLedReadMode = LED_mode * 3

    # ADC Configuration
    def setADCRange(self, ADC_range):
        # ADC range: set the range of the conversion
        # Options: 2048, 4096, 8192, 16384
        # Current draw: 7.81pA. 15.63pA, 31.25pA, 62.5pA per LSB.
        if ADC_range == 2048:
            range = MAX30105_ADCRANGE_2048
        elif ADC_range == 4096:
            range = MAX30105_ADCRANGE_4096
        elif ADC_range == 8192:
            range = MAX30105_ADCRANGE_8192
        elif ADC_range == 16384:
            range = MAX30105_ADCRANGE_16384
        else:
            raise ValueError('Wrong ADC range:{0}!'.format(ADC_range))
            
        self.set_bitMask(MAX30105_PARTICLECONFIG,
                         MAX30105_ADCRANGE_MASK,
                         range)

    # Sample Rate Configuration
    def setSampleRate(self, sample_rate):
        TAG = 'setSampleRate'
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
            
        self.set_bitMask(MAX30105_PARTICLECONFIG,
                         MAX30105_SAMPLERATE_MASK,
                         sr)
        
        logger.debug("(%s) Setting sample rate to %d.", TAG, sample_rate)
        
        # Store the sample rate and recompute the acq. freq.
        self._sampleRate = sample_rate
        self.updateAcquisitionFrequency()
    
    # Pulse width Configuration
    def setPulseWidth(self, pulse_width):
        TAG = 'setPulseWidth'
        # Pulse width of LEDs: The longer the pulse width the longer range of
        # detection. At 69us and 0.4mA it's about 2 inches,
        # at 411us and 0.4mA it's about 6 inches.
        if pulse_width == 69:
            pw = MAX30105_PULSEWIDTH_69
        elif pulse_width == 118:
            pw = MAX30105_PULSEWIDTH_118
        elif pulse_width == 215:
            pw = MAX30105_PULSEWIDTH_215
        elif pulse_width == 411:
            pw = MAX30105_PULSEWIDTH_411
        else:
            raise ValueError('Wrong pulse width:{0}!'.format(pulse_width))
        self.set_bitMask(MAX30105_PARTICLECONFIG,
                         MAX30105_PULSEWIDTH_MASK,
                         pw)
        
        logger.debug("(%s) Setting pulse width to %d.", TAG, pulse_width)
        
        # Store the pulse width
        self._pulseWidth = pw
    
    # LED Pulse Amplitude Configuration methods
    def setActiveLEDsAmplitude(self, amplitude):
        if (self._activeLEDs > 0):
            self.setPulseAmplitudeRed(amplitude)
        if (self._activeLEDs > 1):
            self.setPulseAmplitudeIR(amplitude)
        if(self._activeLEDs > 2):
            self.setPulseAmplitudeGreen(amplitude)
        
    def setPulseAmplitudeRed(self, amplitude):
        self.i2c_set_register(MAX30105_LED1_PULSEAMP, amplitude)
        
    def setPulseAmplitudeIR(self, amplitude):
        self.i2c_set_register(MAX30105_LED2_PULSEAMP, amplitude)
        
    def setPulseAmplitudeGreen(self, amplitude):
        self.i2c_set_register(MAX30105_LED3_PULSEAMP, amplitude)
    
    def setPulseAmplitudeProximity(self, amplitude):
        self.i2c_set_register(MAX30105_LED_PROX_AMP, amplitude)
    
    def setProximityThreshold(self, threshMSB):
        # Set the IR ADC count that will trigger the beginning of particle-
        # sensing mode.The threshMSB signifies only the 8 most significant-bits
        # of the ADC count. (datasheet page 24)
        self.i2c_set_register(MAX30105_PROXINTTHRESH, threshMSB)

    # FIFO averaged samples number Configuration
    def setFIFOAverage(self, number_of_samples):
        TAG = 'setFIFOAverage'
        # FIFO sample avg: set the number of samples to be averaged by the chip.
        # Options: MAX30105_SAMPLEAVG_1, 2, 4, 8, 16, 32
        if number_of_samples == 1:
            ns = MAX30105_SAMPLEAVG_1
        elif number_of_samples == 2:
            ns = MAX30105_SAMPLEAVG_2
        elif number_of_samples == 4:
            ns = MAX30105_SAMPLEAVG_4
        elif number_of_samples == 8:
            ns = MAX30105_SAMPLEAVG_8
        elif number_of_samples == 16:
            ns = MAX30105_SAMPLEAVG_16
        elif number_of_samples == 32:
            ns = MAX30105_SAMPLEAVG_32
        else:
            raise ValueError(
                'Wrong number of samples:{0}!'.format(number_of_samples))
        self.set_bitMask(MAX30105_FIFOCONFIG, MAX30105_SAMPLEAVG_MASK, ns)
        
        logger.debug("(%s) Setting FIFO avg samples to %d.", TAG,
                     number_of_samples)
        
        # Store the number of averaged samples and recompute the acq. freq.
        self._sampleAvg = number_of_samples
        self.updateAcquisitionFrequency()
        
    def updateAcquisitionFrequency(self):
        TAG = 'updateAcquisitionFrequency'
        if (None in [self._sampleRate, self._sampleAvg]):
            logger.debug("(%s) Unable to compute acq freq..", TAG)
            return
        else:
            self._acqFrequency = self._sampleRate / self._sampleAvg
            from math import ceil
            # Compute the time interval to wait before taking a good measure
            # (see note in setSampleRate() method)
            self._acqFrequencyInv = int(ceil(1000/self._acqFrequency))
            logger.info("(%s) Acq. frequency: %f Hz", TAG, self._acqFrequency)
            logger.info("(%s) Acq. period: %f ms", TAG, self._acqFrequencyInv)
    
    def getAcquisitionFrequency(self):
        return self._acqFrequency
    
    def clearFIFO(self):
        # Resets all points to start in a known state
        # Datasheet page 15 recommends clearing FIFO before beginning a read
        self.i2c_set_register(MAX30105_FIFOWRITEPTR, 0)
        self.i2c_set_register(MAX30105_FIFOOVERFLOW, 0)
        self.i2c_set_register(MAX30105_FIFOREADPTR, 0)
        
    def enableFIFORollover(self):
        # FIFO rollover: enable to allow FIFO tro wrap/roll over
        self.set_bitMask(MAX30105_FIFOCONFIG,
                         MAX30105_ROLLOVER_MASK,
                         MAX30105_ROLLOVER_ENABLE)
    
    def disableFIFORollover(self):
        # FIFO rollover: disable to disallow FIFO tro wrap/roll over
        self.set_bitMask(MAX30105_FIFOCONFIG,
                         MAX30105_ROLLOVER_MASK,
                         MAX30105_ROLLOVER_DISABLE)
    
    def setFIFOAlmostFull(self, number_of_samples):
        # Set number of samples to trigger the almost full interrupt (page 18)
        # Power on default is 32 samples. Note it is reverse: 0x00 is 
        # 32 samples, 0x0F is 17 samples
        self.set_bitMask(MAX30105_FIFOCONFIG,
                         MAX30105_A_FULL_MASK,
                         number_of_samples)
    
    def getWritePointer(self):
        # Read the FIFO Write Pointer from the register
        wp = self.i2c_read_register(MAX30105_FIFOWRITEPTR)
        return wp
    
    def getReadPointer(self):
        # Read the FIFO Read Pointer from the register
        wp = self.i2c_read_register(MAX30105_FIFOREADPTR)
        return wp
    
    # Die Temperature method: returns the temperature in C
    def readTemperature(self):
        # DIE_TEMP_RDY interrupt must be enabled
        # Config die temperature register to take 1 temperature sample
        self.i2c_set_register(MAX30105_DIETEMPCONFIG, 0x01)
        
        # Poll for bit to clear, reading is then complete
        reading = ord(self.i2c_read_register(MAX30105_INTSTAT2))
        sleep_ms(100);
        while ((reading & MAX30105_INT_DIE_TEMP_RDY_ENABLE) > 0):
            reading = ord(self.i2c_read_register(MAX30105_INTSTAT2))
            sleep_ms(1);
        
        # Read die temperature register (integer)
        tempInt = ord(self.i2c_read_register(MAX30105_DIETEMPINT))
        # Causes the clearing of the DIE_TEMP_RDY interrupt
        tempFrac = ord(self.i2c_read_register(MAX30105_DIETEMPFRAC))
        
        # Calculate temperature (datasheet pg. 23)
        return float(tempInt) + (float(tempFrac) * 0.0625)

    def setPROXINTTHRESH(self, val):
        # Set the PROX_INT_THRESH (see proximity function on datasheet, pag 10)
        self.i2c_set_register(MAX30105_PROXINTTHRESH, val)
    
    # DeviceID and Revision methods
    def readPartID(self):
        # Load the Device ID from the register
        part_id = self.i2c_read_register(MAX30105_PARTID)
        return part_id
    
    def checkPartID(self):
        # Checks the correctness of the Device ID
        part_id = ord(self.readPartID())
        return part_id == MAX_30105_EXPECTEDPARTID
    
    def getRevisionID(self):
        # Load the Revision ID from the register
        rev_id = self.i2c_read_register(MAX30105_REVISIONID)
        return ord(rev_id)

    # Time slots management for multi-LED operation mode
    def enableSlot(self, slotNumber, device):
        # In multi-LED mode, each sample is split into up to four time slots, 
        # SLOT1 through SLOT4. These control registers determine which LED is
        # active in each time slot. (datasheet pag 22)
        # Devices are SLOT_RED_LED or SLOT_RED_PILOT (proximity)
        # Assigning a SLOT_RED_LED will pulse LED
        # Assigning a SLOT_RED_PILOT will detect the proximity
        if   (slotNumber == 1):
            self.bitMask(MAX30105_MULTILEDCONFIG1, MAX30105_SLOT1_MASK, device)
        elif (slotNumber == 2):
            self.bitMask(MAX30105_MULTILEDCONFIG1, MAX30105_SLOT2_MASK, device << 4)
        elif (slotNumber == 3):
            self.bitMask(MAX30105_MULTILEDCONFIG2, MAX30105_SLOT3_MASK, device)
        elif (slotNumber == 4):
            self.bitMask(MAX30105_MULTILEDCONFIG2, MAX30105_SLOT4_MASK, device << 4)
        else:
            raise ValueError('Wrong slot number:{0}!'.format(slotNumber))

    def disableSlots(self):
        # Clear all the slots assignments
        self.i2c_set_register(MAX30105_MULTILEDCONFIG1, 0)
        self.i2c_set_register(MAX30105_MULTILEDCONFIG2, 0)

    # Low-level I2C Communication
    def i2c_read_register(self, REGISTER, n_bytes=1):
        self._i2c.writeto(self._address, bytearray([REGISTER]))
        return self._i2c.readfrom(self._address, n_bytes)

    def i2c_set_register(self, REGISTER, VALUE):
        self._i2c.writeto(self._address, bytearray([REGISTER, VALUE]))
        return

    # Given a register, read it, mask it, and then set the thing
    def set_bitMask(self, REGISTER, MASK, NEW_VALUES):
        newCONTENTS = (ord(self.i2c_read_register(REGISTER)) & MASK) | NEW_VALUES
        self.i2c_set_register(REGISTER, newCONTENTS)
        return
    
    # Given a register, read it and mask it
    def bitMask(self, reg, slotMask, thing):
        originalContents = ord(self.i2c_read_register(reg))
        originalContents = originalContents & slotMask
        self.i2c_set_register(reg, originalContents | thing)

    def FIFO_bytes_to_int(self, FIFO_bytes):
        value = unpack(">i", b'\x00' + FIFO_bytes)
        return (value[0] & 0x3FFFF) >> self._pulseWidth
    
    # Returns how many samples are available
    def available(self):
        numberOfSamples = len(self.sense.red)
        return numberOfSamples
    
    # Get a new red value
    def getRed(self):
        # Check the sensor for new data for 250ms
        if (self.safeCheck(250)):
            return self.sense.red.popHead()
        else:
            # Sensor failed to find new data
            return 0
    
    # Get a new IR value
    def getIR(self):
        # Check the sensor for new data for 250ms
        if (self.safeCheck(250)):
            return self.sense.IR.popHead()
        else:
            # Sensor failed to find new data
            return 0
    
    # Get a new green value
    def getGreen(self):
        # Check the sensor for new data for 250ms
        if (self.safeCheck(250)):
            return self.sense.green.popHead()
        else:
            # Sensor failed to find new data
            return 0
    
    # Note: the following 3 functions are the equivalent of using 'getFIFO'
    # methods of the SparkFun library
    # Pops the next red value in storage (if available)
    def popRedFromStorage(self):
        if (len(self.sense.red) == 0):
            return 0
        else:
            return self.sense.red.pop()
    
    # Pops the next IR value in storage (if available)
    def popIRFromStorage(self):
        if (len(self.sense.IR) == 0):
            return 0
        else:
            return self.sense.IR.pop()
        
    # Pops the next green value in storage (if available)
    def popGreenFromStorage(self):
        if (len(self.sense.green) == 0):
            return 0
        else:
            return self.sense.green.pop()
    
    # (useless - for comparison purposes only)
    def nextSample(self):
        if(self.available()):
            # With respect to the SparkFun library, using a deque object 
            # allows us to avoid manually advancing of the tail
            return True

    # Polls the sensor for new data
    def check(self):
        TAG = "check"
        
        # Call continuously to poll the sensor for new data.
        readPointer = ord(self.getReadPointer())
        writePointer = ord(self.getWritePointer())
        numberOfSamples = 0
        
        # Do we have new data?
        if (readPointer != writePointer):
            # Calculate the number of readings we need to get from sensor
            numberOfSamples = writePointer - readPointer
            
            # Wrap condition (return to the beginning of 32 samples)
            if (numberOfSamples < 0):
                numberOfSamples += 32
            
            for i in range(numberOfSamples):
                # Read a number of bytes equal to activeLEDs*3 (= 1 sample)
                fifo_bytes = self.i2c_read_register(MAX30105_FIFODATA,
                                                    self._multiLedReadMode)
                        
                # Convert the readings from bytes to integers, depending
                # on the number of active LEDs
                if (self._activeLEDs > 0):
                    self.sense.red.append(
                        self.FIFO_bytes_to_int(fifo_bytes[0:3])
                    )
                
                if (self._activeLEDs > 1):
                    self.sense.IR.append(
                        self.FIFO_bytes_to_int(fifo_bytes[3:6])
                    )
                
                if (self._activeLEDs > 2):
                    self.sense.green.append(
                        self.FIFO_bytes_to_int(fifo_bytes[6:9])
                    )
                    
                return True
        
        else:
            return False
    
    # Check for new data but give up after a certain amount of time
    def safeCheck(self, maxTimeToCheck):
        markTime = ticks_ms()
        while(True):
            if (ticks_diff(ticks_ms(), markTime) > maxTimeToCheck):
                # Timeout reached
                return False
            if (self.check() == True):
                # new data found
                return True
            sleep_ms(1)
