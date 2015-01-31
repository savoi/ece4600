#!/usr/bin/python

import random
import serial
import time

class ArduinoComm:

	PIN1_NUM = 21
	GPIO1_ROOT = '/sys/class/gpio'
	GPIO1_PIN = '{0}/gpio{1}'.format(GPIO1_ROOT, PIN1_NUM)
	GPIO1_DIR = '{0}/direction'.format(GPIO1_PIN)
	GPIO1_VAL = '{0}/value'.format(GPIO1_PIN)

	PIN2_NUM = 19
	GPIO2_ROOT = '/sys/class/gpio'
	GPIO2_PIN = '{0}/gpio{1}'.format(GPIO2_ROOT, PIN2_NUM)
	GPIO2_DIR = '{0}/direction'.format(GPIO2_PIN)
	GPIO2_VAL = '{0}/value'.format(GPIO2_PIN)
	
	ser = None

	def init(self):
		self.ser = serial.Serial('/dev/ttymxc3',115200,timeout=1)
		self.ser.flushOutput()
		print 'Serial connected'

	def blinkLED(self, gpio_val):
		for num in range(100): 
			self.writeGPIO(gpio_val,1)
			time.sleep(random.random())
			self.writeGPIO(gpio_val,0)
			time.sleep(random.random())

	# Perform GPIO handshake and send data over serial line
	def sendData(self, data):
		# Pull pin 2 low
		if(self.getGPIODirection(self.GPIO2_DIR) == 'out'):
			if(self.readGPIO(self.GPIO2_VAL) == '1'):
				self.pullPinLow(self.GPIO2_VAL)
				self.writeData(self.ser, 1)
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
