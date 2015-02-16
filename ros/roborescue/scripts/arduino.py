#!/usr/bin/python

import random
import rospy
import serial
import time

from std_msgs.msg import String

class ArduinoComm:

	SERIAL_PATH = '/dev/ttymxc3'
	GPIO_ROOT = '/sys/class/gpio'
	
	BAUD_RATE = 115200
	LOOP_RATE = 10				# loop freq in hertz
	PACKET_SIZE = 5				# in bytes
	MAX_READ_SIZE = 200			# in bytes
	ARDUINO_PIN = 136 			# Arduino pin 30 / GPIO pin 136
	LINUX_PIN = 137				# Arduino pin 31 / GPIO pin 137
	
	ARDUINO_PIN_PATH = '{0}/gpio{1}'.format(GPIO_ROOT, ARDUINO_PIN)
	ARDUINO_PIN_DIR = '{0}/direction'.format(ARDUINO_PIN_PATH)
	ARDUINO_PIN_VAL = '{0}/value'.format(ARDUINO_PIN_PATH)

	LINUX_PIN_PATH = '{0}/gpio{1}'.format(GPIO_ROOT, LINUX_PIN)
	LINUX_PIN_DIR = '{0}/direction'.format(LINUX_PIN_PATH)
	LINUX_PIN_VAL = '{0}/value'.format(LINUX_PIN_PATH)
	
	ser = None
	pub = None			# ros publisher
	sub = None			# ros subscriber
	msgs = ['']

	def __init__(self):
		rospy.init_node('arduino', anonymous=False)
		self.rate = rospy.Rate(self.LOOP_RATE)
		self.pub = rospy.Publisher('status_msgs', String)
		self.sub = rospy.Subscriber('control_msgs', String, self.callback)
		self.ser = serial.Serial(self.SERIAL_PATH,self.BAUD_RATE,timeout=1)
		self.ser.flushOutput()
		rospy.loginfo('Serial connected')
		self.initHandshake()
		rospy.loginfo('Done initial handshake with Arduino')

	def callback(self, data):
		self.msgs[0] = data.data
		
	# Perform handshake with Arduino to initiate communication
	def initHandshake(self):
		# Confirm pin directions
		if(self.getGPIODirection(self.ARDUINO_PIN_DIR) == 'out'):
			self.setGPIODirection(self.ARDUINO_PIN_DIR, 'in')
		if(self.getGPIODirection(self.LINUX_PIN_DIR) == 'in'):
			self.setGPIODirection(self.LINUX_PIN_DIR, 'out')
			
		# perform handshake
		rospy.loginfo('Waiting for Arduino pin to go high...')
		while(self.readGPIO(self.ARDUINO_PIN_VAL) == '0'):
			pass
		rospy.loginfo('Pulling Linux pin high...')
		self.pullPinHigh(self.LINUX_PIN_VAL)				# Bring Linux pin high
		rospy.loginfo('Waiting for Arduino pin to go low...')
		while(self.readGPIO(self.ARDUINO_PIN_VAL) == '1'):	# Wait for Arduino pin to go low
			pass
		rospy.loginfo('Pulling Linux pin low')
		self.pullPinLow(self.LINUX_PIN_VAL)			# Pull Linux pin low
		time.sleep(5)								# Settle time
		rospy.loginfo('--> init handshake complete')

	# Send and receive data at given frequency
	def start(self):
		if(self.getGPIODirection(self.LINUX_PIN_DIR) == 'out'):
			rospy.loginfo('Waiting for Arduino pin to go low...')
			while(self.readGPIO(self.ARDUINO_PIN_VAL) == '1'):
				pass
			rospy.loginfo('Pulling Linux pin high')
			self.pullPinHigh(self.LINUX_PIN_VAL)
			rospy.loginfo('Waiting for Arduino pin to go high...')
			while(self.readGPIO(self.ARDUINO_PIN_VAL) == '0'):
				pass
			rospy.loginfo('Pulling Linux pin low')
			self.pullPinLow(self.LINUX_PIN_VAL)
			# send data
			rospy.loginfo('sending data...')
			rospy.loginfo('DATA TO SEND: {0}'.format(self.msgs[0]))
			self.writeData(str(self.msgs[0]))
			rospy.loginfo('Waiting for Arduino pin to go low...')
			while(self.readGPIO(self.ARDUINO_PIN_VAL) == '1'):
				pass
			# receive data
			data = self.readData(self.ser, self.MAX_READ_SIZE)
			if data:
				rospy.loginfo('--> DATA: {0}'.format(data.encode('hex')))
				self.pub.publish(data)
		self.rate.sleep()

	# Perform GPIO handshake and send data over serial line
	def sendData(self, data):
		if(self.getGPIODirection(self.LINUX_PIN_DIR) == 'out'):
			rospy.loginfo('Waiting for Arduino pin to go low...')
			while(self.readGPIO(self.ARDUINO_PIN_VAL) == '1'):
				pass
			rospy.loginfo('Pulling Linux pin high')
			self.pullPinHigh(self.LINUX_PIN_VAL)
			rospy.loginfo('Waiting for Arduino pin to go high...')
			while(self.readGPIO(self.ARDUINO_PIN_VAL) == '0'):
				pass
			rospy.loginfo('Pulling Linux pin low')
			self.pullPinLow(self.LINUX_PIN_VAL)
			rospy.loginfo('sending data...')
			rospy.loginfo('DATA TO SEND: {0:b}'.format(data))
			self.writeData(data)
		else:
			# throw error
			pass

	# Perform GPIO handshake and receive data over serial line
	def recvData(self):
		# Read pin to see if pulled low
		rospy.loginfo('data receive handshake')
		if(self.getGPIODirection(self.LINUX_PIN_DIR) == 'out'):
			rospy.loginfo('Waiting for Arduino pin to go high...')
			while(self.readGPIO(self.ARDUINO_PIN_VAL) == '0'):
				pass
			rospy.loginfo('Pulling Linux pin high')
			self.pullPinHigh(self.LINUX_PIN_VAL)
			rospy.loginfo('Waiting for Arduino pin to go low...')
			while(self.readGPIO(self.ARDUINO_PIN_VAL) == '1'):
				pass
			rospy.loginfo('Waiting for Arduino pin to go high...')
			while(self.readGPIO(self.ARDUINO_PIN_VAL) == '0'):
				pass
			rospy.loginfo('Pulling Linux pin low')
			self.pullPinLow(self.LINUX_PIN_VAL)
			data = self.readData(self.ser, self.PACKET_SIZE)
			rospy.loginfo('--> DATA: {0}'.format(data.encode('hex')))
			self.pub.publish(data)
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
		return direction
		
	# Set the GPIO pin to either input or output
	def setGPIODirection(self, gpiopath, direction):
		f = open(gpiopath, 'w')
		f.write(str(direction))
		f.close()

	def pullPinLow(self, gpio_val):
		self.writeGPIO(gpio_val, '0')

	def pullPinHigh(self, gpio_val):
		self.writeGPIO(gpio_val, '1')

	def writeData(self, data):
		self.ser.write(data)

	def readData(self, ser, bytes):
		data = ser.read(size=bytes)
		return data

if __name__ == "__main__":
	ac = ArduinoComm()
	while not rospy.is_shutdown():
		ac.start()
