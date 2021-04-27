# main.py

from lib.MAX30102 import MAX30102

if __name__ == '__main__':
	sensor = MAX30102()  # Loads default ESP32 I2C configuration
	# sensor = ParticleSensor(i2cHexAddress = 0x57)
	# sensor = ParticleSensor(i2cHexAddress = 0x57, i2c = i2cInstance)

	print("Setting up sensor.", '\n')
	# Setup the sensor with default configuration
	sensor.setup_sensor()
 	print(sensor.readTemperature())