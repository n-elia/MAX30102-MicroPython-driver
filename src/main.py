# main.py

from MAX30102 import MAX30102

def main():
	sensor = MAX30102()  # Loads default ESP32 I2C configuration
	# sensor = ParticleSensor(i2cHexAddress = 0x57)
	# sensor = ParticleSensor(i2cHexAddress = 0x57, i2c = i2cInstance)

	print("Setting up sensor.", '\n')
	# Setup the sensor with default configuration
	sensor.setup_sensor()
	sensor.setPulseAmplitudeRed(255)
	sensor.setPulseAmplitudeGreen(0x00)


if __name__ == '__main__':
	main()
