#!/usr/bin/python

import random
import rospy
import serial
import time

from std_msgs.msg import String

def callback(data):
	rospy.loginfo(rospy.get_caller_id() + 'msg recvd: %s', data.data)

# Executed when a message is received
def subscribe():
	rospy.init_node('arduino_fwd', anonymous=False)
	rospy.Subscriber('control_msgs', String, callback)
	rospy.spin()

class ArduinoComm:

	SERIAL_PATH = '/dev/ttymxc3'
	GPIO_ROOT = '/sys/class/gpio'
	
	BAUD_RATE = 115200
	ARDUINO_PIN = 136 			# Arduino pin 30 / GPIO pin 136
	LINUX_PIN = 137				# Arduino pin 31 / GPIO pin 137
	
	ARDUINO_PIN_PATH = '{0}/gpio{1}'.format(GPIO_ROOT, ARDUINO_PIN)
	ARDUINO_PIN_DIR = '{0}/direction'.format(ARDUINO_PIN_PATH)
	ARDUINO_PIN_VAL = '{0}/value'.format(ARDUINO_PIN_PATH)

	LINUX_PIN_PATH = '{0}/gpio{1}'.format(GPIO_ROOT, LINUX_PIN)
	LINUX_PIN_DIR = '{0}/direction'.format(LINUX_PIN_PATH)
	LINUX_PIN_VAL = '{0}/value'.format(LINUX_PIN_PATH)
	
	ser = None

	def __init__(self):
		self.ser = serial.Serial(self.SERIAL_PATH,self.BAUD_RATE,timeout=1)
		self.ser.flushOutput()
		print 'Serial connected'
		self.initHandshake()
		print 'Done initial handshake with Arduino'
		
	# Perform handshake with Arduino to initiate communication
	def initHandshake(self):
		# Confirm pin directions
		if(self.getGPIODirection(self.ARDUINO_PIN_DIR) == 'out'):
			self.setGPIODirection(self.ARDUINO_PIN_DIR, 'in')
		if(self.getGPIODirection(self.LINUX_PIN_DIR) == 'in'):
			self.setGPIODirection(self.LINUX_PIN_DIR, 'out')
			
		# perform handshake
		print 'Waiting for Arduino pin to go high...'
		while(self.readGPIO(self.ARDUINO_PIN_VAL) == '0'):
			pass
		print 'Pulling Linux pin high...'
		self.pullPinHigh(self.LINUX_PIN_VAL)				# Bring Linux pin high
		print 'Waiting for Arduino pin to go low...'
		while(self.readGPIO(self.ARDUINO_PIN_VAL) == '1'):	# Wait for Arduino pin to go low
			pass
		print 'Pulling Linux pin low'
		self.pullPinLow(self.LINUX_PIN_VAL)			# Pull Linux pin low
		time.sleep(5)										# Settle time
		print '--> init handshake complete'

	# Perform GPIO handshake and send data over serial line
	def sendData(self, data):
		if(self.getGPIODirection(self.LINUX_PIN_DIR) == 'out'):
			print 'Waiting for Arduino pin to go low...'
			while(self.readGPIO(self.ARDUINO_PIN_VAL) == '1'):
				pass
			print 'Pulling Linux pin high'
			self.pullPinHigh(self.LINUX_PIN_VAL)
			print 'Waiting for Arduino pin to go high...'
			while(self.readGPIO(self.ARDUINO_PIN_VAL) == '0'):
				pass
			print 'Pulling Linux pin low'
			self.pullPinLow(self.LINUX_PIN_VAL)
			print 'sending data...'
			print 'DATA TO SEND: {0:b}'.format(data)
			self.writeData(data)
		else:
			# throw error
			pass

	# Perform GPIO handshake and receive data over serial line
	def recvData(self):
		# Read pin to see if pulled low
		print 'data receive handshake'
		if(self.getGPIODirection(self.LINUX_PIN_DIR) == 'out'):
			print 'Waiting for Arduino pin to go high...'
			while(self.readGPIO(self.ARDUINO_PIN_VAL) == '0'):
				pass
			print 'Pulling Linux pin high'
			self.pullPinHigh(self.LINUX_PIN_VAL)
			print 'Waiting for Arduino pin to go low...'
			while(self.readGPIO(self.ARDUINO_PIN_VAL) == '1'):
				pass
			print 'Waiting for Arduino pin to go high...'
			while(self.readGPIO(self.ARDUINO_PIN_VAL) == '0'):
				pass
			print 'Pulling Linux pin low'
			self.pullPinLow(self.LINUX_PIN_VAL)
			data = self.readData(self.ser, 2)
			print '--> DATA: {0}'.format(data)
		else:
			# throw error
			pass

	def readGPIO(self, gpio_path):
		with open(gpio_path, 'r') as f:
			data = f.read()
			val = data.split('\n')[0]
			return val

	def writeGPIO(self, gpio_val ,value):
		with open(gpio_val, 'w') as f:
			f.write(str(value))

	def getGPIODirection(self, dirpath):
		f = open(dirpath, 'r')
		result = f.read()
		direction = result.split('\n')[0]
		#print 'GPIO direction: {0}'.format(direction)
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
		data = ser.read(bytes)
		return data

def main():
	ac = ArduinoComm()
	while(True):
		ac.recvData()

if __name__ == "__main__":
	#main()
	subscribe()
