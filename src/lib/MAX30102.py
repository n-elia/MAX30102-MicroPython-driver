from ustruct import unpack
from machine import SoftI2C, Pin
from utime import sleep_ms

import logging

# Setup of logger
logging.basicConfig(level=logging.DEBUG)
logger = logging.getLogger("MAX30102")

# TODO: parametrize the i2c just to avoid being hardcoded for esp32 only
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
        self._led_mode = None
        self._pulse_width_set = None
        
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
    def setLEDMode(self, LED_MODE):
        # Set LED mode: select which LEDs are used for sampling 
        # Options: RED only, RED + IR only, or ALL (datasheet pag. 19)        
        if LED_MODE == MAX30105_MODE_REDONLY:
            self.set_bitMask(MAX30105_MODECONFIG,
                             MAX30105_MODE_MASK,
                             MAX30105_MODE_REDONLY)
            self.i2c_set_register(MAX30105_LED1_PULSEAMP, 
                                  MAX30105_PULSEAMP_MEDIUM) # Red
        elif LED_MODE == MAX30105_MODE_REDIRONLY:
            self.set_bitMask(MAX30105_MODECONFIG,
                             MAX30105_MODE_MASK,
                             MAX30105_MODE_REDIRONLY)
            self.i2c_set_register(MAX30105_LED1_PULSEAMP, 
                                  MAX30105_PULSEAMP_MEDIUM) # Red
            self.i2c_set_register(MAX30105_LED2_PULSEAMP, 
                                  MAX30105_PULSEAMP_MEDIUM) # IR
        elif LED_MODE == MAX30105_MODE_MULTILED:
            self.set_bitMask(MAX30105_MODECONFIG,
                             MAX30105_MODE_MASK,
                             MAX30105_MODE_MULTILED)
            self.i2c_set_register(MAX30105_LED1_PULSEAMP, 
                                  MAX30105_PULSEAMP_MEDIUM) # Red
            self.i2c_set_register(MAX30105_LED2_PULSEAMP, 
                                  MAX30105_PULSEAMP_MEDIUM) # IR
            self.i2c_set_register(MAX30105_LED3_PULSEAMP, 
                                  MAX30105_PULSEAMP_MEDIUM) # Green
            # FIXME move these elsewhere
            self.i2c_set_register(0x11, 0b00100001) #mulitledconfig1
            self.i2c_set_register(0x12, 0b00000011) #mutliledconfig2
        else:
            raise ValueError('Wrong LED mode:{0}!'.format(LED_MODE))
            
        # Store the LED mode
        self._led_mode = LED_MODE

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
    
    # Pulse width Configuration
    def setPulseWidth(self, pulse_width):
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
        
        # Store the pulse width
        self._pulse_width_set = pw
    
    # LED Pulse Amplitude Configuration methods
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

    def CreateImage(self, value):
        unit = (2 ** (18 - self._pulse_width_set)) // (250)
        image_p1 = (value // (unit * 50)) * (str(9) * 5)
        image_p2 = ((value % (unit * 50)) // (unit * 10)) * str(9)
        points = (((value % (unit * 50)) % (unit * 10))) // unit
        if points > 0:
            image_p3 = str(points)
        else:
            image_p3 = ""
        image_p4 = ((25) - len(image_p1 + image_p2 + image_p3)) * str(0)
        tmp_image = image_p1 + image_p2 + image_p3 + image_p4
        return ':'.join([tmp_image[i:i+5] for i in range(0, len(tmp_image), 5)])

    def i2c_read_register(self, REGISTER, n_bytes=1):
        self._i2c.writeto(self._address, bytearray([REGISTER]))
        return self._i2c.readfrom(self._address, n_bytes)

    def i2c_set_register(self, REGISTER, VALUE):
        self._i2c.writeto(self._address, bytearray([REGISTER, VALUE]))
        return

    def set_bitMask(self, REGISTER, MASK, NEW_VALUES):
        newCONTENTS = (ord(self.i2c_read_register(REGISTER)) & MASK) | NEW_VALUES
        self.i2c_set_register(REGISTER, newCONTENTS)
        return
  
    def bitMask(self, reg, slotMask, thing):
        originalContents = ord(self.i2c_read_register(reg))
        originalContents = originalContents & slotMask
        self.i2c_set_register(reg, originalContents | thing)

    def setup_sensor(self, LED_MODE=3,
                     LED_POWER=MAX30105_PULSEAMP_LOW,
                     PULSE_WIDTH=118):
        # Reset the sensor's registers from previous configurations
        self.softReset()
        
        # Set the Pulse Width to the default value of 118
        self.setPulseWidth(PULSE_WIDTH)
        
        # Set the LED mode to the default value of RED + IR
        self.setLEDMode(MAX30105_MODE_REDIRONLY)

        # Set the sample rate to the default value of 800
        self.setSampleRate(800)

        # Set the ADC range to default value of 16384
        self.setADCRange(16384)

        # Set the number of samples to be averaged by the chip to 16
        self.setFIFOAverage(16)

        # Allow FIFO queue to wrap/roll over
        self.enableFIFORollover()

        # clears the FIFO
        self.clearFIFO()
        
    

    def FIFO_bytes_to_int(self, FIFO_bytes):
        value = unpack(">i", b'\x00' + FIFO_bytes)
        return (value[0] & 0x3FFFF) >> self._pulse_width_set

    def read_sensor_multiLED(self, pointer_position):
        sleep_ms(25)
        self.i2c_set_register(0x06, pointer_position) #mutliled
        fifo_bytes = self.i2c_read_register(MAX30105_FIFODATA, self._led_mode * 3) #mode_mult

        red_int = self.FIFO_bytes_to_int(fifo_bytes[0:3])
        IR_int = self.FIFO_bytes_to_int(fifo_bytes[3:6])
        green_int = self.FIFO_bytes_to_int(fifo_bytes[6:9])
        
        #print("[Red:", red_int, " IR:", IR_int, " G:", green_int, "]", sep='')
        
        return red_int, IR_int, green_int
    
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
        return rev_id
