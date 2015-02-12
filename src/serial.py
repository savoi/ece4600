#!/usr/bin/python

import random
import serial
import time

class ArduinoComm:

	SERIAL_PATH = '/dev/ttymxc3'
	GPIO_ROOT = '/sys/class/gpio'
	
	BAUD_RATE = 115200
	ARDUINO_PIN = 21
	LINUX_PIN = 19
	
	ARDUINO_PIN_PATH = '{0}/gpio{1}'.format(GPIO_ROOT, ARDUINO_PIN)
	ARDUINO_PIN_DIR = '{0}/direction'.format(ARDUINO_PIN_PATH)
	ARDUINO_PIN_VAL = '{0}/value'.format(ARDUINO_PIN_PATH)

	LINUX_PIN_PATH = '{0}/gpio{1}'.format(GPIO_ROOT, LINUX_PIN)
	LINUX_PIN_DIR = '{0}/direction'.format(LINUX_PIN_PATH)
	LINUX_PIN_VAL = '{0}/value'.format(LINUX_PIN_PATH)
	
	ser = None

	def init(self):
		self.ser = serial.Serial(self.SERIAL_PATH,self.BAUD_RATE,timeout=1)
		self.ser.flushOutput()
		print 'Serial connected'
		self.initHandshake()
		print 'Done initial handshake with Arduino'
		
	# Perform handshake with Arduino to initiate communication
	def initHandshake(self):
		# Confirm pin directions
		if(self.getGPIODirection(ARDUINO_PIN_PATH) == 'out'):
			self.setGPIODirection(ARDUINO_PIN_PATH, 'in')
		if(self.getGPIODirection(LINUX_PIN_PATH) == 'in'):
			self.setGPIODirection(LINUX_PIN_PATH, 'out')
			
		# perform handshake
		while(readGPIO(self.ARDUINO_PIN_PATH) == '0'):
			pass	# busy wait
		pullPinHigh(self.LINUX_PIN_PATH)				# Bring Linux pin high
		while(readGPIO(ARDUINO_PIN_PATH) == '1'):		# Wait for Arduino pin to go low
			pass	# busy wait
		pullPinLow(self.LINUX_PIN_PATH)					# Pull Linux pin low
		sleep(5)										# Settle time

	def blinkLED(self, gpio_val):
		for num in range(100): 
			self.writeGPIO(gpio_val,1)
			time.sleep(random.random())
			self.writeGPIO(gpio_val,0)
			time.sleep(random.random())

	# Perform GPIO handshake and send data over serial line
	def sendData(self, data):
		if(self.getGPIODirection(self.LINUX_PIN_DIR) == 'out'):
			if(self.readGPIO(self.ARDUINO_PIN_PATH) == '1'):
				self.pullPinHigh(self.LINUX_PIN_PATH)
				self.writeData(self.ser, data)
				self.pullPinLow(self.LINUX_PIN_PATH)
				print data
			else:
				# no data ready
				pass
		else:
			# throw error
			pass

	# Perform GPIO handshake and receive data over serial line
	def recvData(self):
		# Read pin to see if pulled low
		if(self.getGPIODirection(self.GPIO1_DIR) == 'in'):
			if(self.readGPIO(self.GPIO1_VAL) == '0'):
				self.pullPinLow(self.GPIO_VAL2)
				data = self.readData(self.ser, 1)
				print data
			else:
				# no data ready
				pass
		else:
			# throw error
			pass

	def readGPIO(self, gpio_path):
		f = open(gpio_path, 'r')
		data = f.read()
		f.close()
		return data

	def writeGPIO(self, led,value):
		f = open(led, 'w')
		f.write(str(value))
		f.close()

	def getGPIODirection(self, dirpath):
		f = open(dirpath, 'r')
		direction = f.read()
		print 'GPIO direction: {0}'.format(direction)
		return direction
		
	# Set the GPIO pin to either input or output
	def setGPIODirection(self, gpiopath, direction):
		f = open(gpiopath, 'w')
		f.write(str(direction))
		f.close()
		print 'GPIO direction set: {0}'.format(direction)

	def pullPinLow(self, gpio_val):
		self.writeGPIO(gpio_val, '0')

	def pullPinHigh(self, gpio_val):
		self.writeGPIO(gpio_val, '1')

	def writeData(self, ser, data):
		self.ser.write(data)

	def readData(self, ser, bytes):
		data = self.ser.read(bytes)
		return data

def main():
	ac = ArduinoComm()
	ac.init()
	ac.blinkLED(ac.GPIO1_VAL)
	ac.recvData()

if __name__ == "__main__":
	main()
