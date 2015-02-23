#!/usr/bin/python

import random
import rospy
import serial
import signal
import time

from bitstring import Bits
from std_msgs.msg import String

class ArduinoComm:

	SERIAL_PATH = '/dev/ttymxc3'
	GPIO_ROOT = '/sys/class/gpio'
	
	BAUD_RATE = 115200
	LOOP_RATE = 10				# loop freq in hertz
	PACKET_SIZE = 5				# in bytes
	MAX_READ_SIZE = 200			# in bytes
	ARDUINO_PIN = 136 			# Arduino pin 30 / GPIO pin 136
	ARDUINO_RESET_PIN = 125		# Arduino D24 / GPIO pin 125 (wired to Arduino RESET)
	LINUX_PIN = 137				# Arduino pin 31 / GPIO pin 137
	HS_TIMEOUT = 20				# Handshake timeout (seconds)
	SERIAL_TIMEOUT = 0.01		# Serial timeout in seconds
	MAX_ARDUINO_RESETS = 3		# Number of times Arduino can be reset due to timeouts
	PUB_QUEUE_SIZE = 10			# Number of messages to queue on status msg topic
	
	ARDUINO_PIN_PATH = '{0}/gpio{1}'.format(GPIO_ROOT, ARDUINO_PIN)
	ARDUINO_PIN_DIR = '{0}/direction'.format(ARDUINO_PIN_PATH)
	ARDUINO_PIN_VAL = '{0}/value'.format(ARDUINO_PIN_PATH)

	ARDUINO_RESET_PIN_PATH = '{0}/gpio{1}'.format(GPIO_ROOT, ARDUINO_RESET_PIN)
	ARDUINO_RESET_PIN_DIR = '{0}/direction'.format(ARDUINO_RESET_PIN_PATH)
	ARDUINO_RESET_PIN_VAL = '{0}/value'.format(ARDUINO_RESET_PIN_PATH)

	LINUX_PIN_PATH = '{0}/gpio{1}'.format(GPIO_ROOT, LINUX_PIN)
	LINUX_PIN_DIR = '{0}/direction'.format(LINUX_PIN_PATH)
	LINUX_PIN_VAL = '{0}/value'.format(LINUX_PIN_PATH)

	def __init__(self):
		self.msgs = ['']
		self.numtimeouts = 0
		rospy.init_node('arduino', anonymous=False)
		self.pub = rospy.Publisher('status_msgs', String, queue_size=self.PUB_QUEUE_SIZE)
		self.sub = rospy.Subscriber('control_msgs', String, self.callback)
		self.rate = rospy.Rate(self.LOOP_RATE)
		signal.signal(signal.SIGALRM, self.timeoutHandler)
		self.arduinoInit()

	def callback(self, data):
		self.msgs[0] = data.data

	# Initializes and runs the arduino code
	def arduinoInit(self):
		self.initpins()
		self.ser = serial.Serial(self.SERIAL_PATH,self.BAUD_RATE,timeout=self.SERIAL_TIMEOUT)
		self.ser.flushOutput()
		rospy.loginfo('Serial connected')
		self.initHandshake()
		self.start()

	# Initialize the GPIO pin directions and values
	def initpins(self):
		if self.getGPIODirection(self.ARDUINO_RESET_PIN_DIR) == 'in':
			self.setGPIODirection(self.ARDUINO_RESET_PIN_DIR, 'out')
		self.pullPinHigh(self.ARDUINO_RESET_PIN_VAL)
		if self.getGPIODirection(self.ARDUINO_PIN_DIR) == 'out':
			self.setGPIODirection(self.ARDUINO_PIN_DIR, 'in')
		if self.getGPIODirection(self.LINUX_PIN_DIR) == 'in':
			self.setGPIODirection(self.LINUX_PIN_DIR, 'out')

	# Handles handshaking timeouts between processors
	def timeoutHandler(self, signum, frame):
		rospy.logerr('Handshaking timeout error')
		self.numtimeouts = self.numtimeouts + 1
		rospy.logerr('Num timeouts: {0}'.format(self.numtimeouts))
		if self.numtimeouts >= self.MAX_ARDUINO_RESETS:
			raise IOError('Handshaking timeout error')
		rospy.logerr('resetting arduino')
		self.pullPinLow(self.ARDUINO_RESET_PIN_VAL)
		rospy.logerr('reinitializing arduino')
		time.sleep(3)
		self.arduinoInit()
		
	# Perform handshake with Arduino to initiate communication
	def initHandshake(self):
		rospy.loginfo('--> starting init handshake')
		self.waitForHigh()
		rospy.loginfo('Pulling Linux pin high...')
		self.pullPinHigh(self.LINUX_PIN_VAL)
		self.waitForLow()
		rospy.loginfo('Pulling Linux pin low')
		self.pullPinLow(self.LINUX_PIN_VAL)
		time.sleep(1)								# Settle time
		rospy.loginfo('--> init handshake complete')

	# Send and receive data at given frequency
	def start(self):
		while not rospy.is_shutdown():
			if(self.getGPIODirection(self.LINUX_PIN_DIR) == 'out'):
				#self.waitForLow()
				rospy.logdebug('Pulling Linux pin high')
				self.pullPinHigh(self.LINUX_PIN_VAL)
				self.waitForHigh()
				rospy.logdebug('Pulling Linux pin low')
				self.pullPinLow(self.LINUX_PIN_VAL)
				# send data
				rospy.loginfo('DATA TO SEND: {0}'.format(self.msgs[0]))
				self.writeData(str(self.msgs[0]))
				self.waitForLow()
				# receive data
				data = self.readData(self.ser, self.MAX_READ_SIZE)
				if data:
					rospy.loginfo('--> DATA RECVD: {0}'.format(data.encode('hex')))
					self.pub.publish(data)
			self.rate.sleep()

	# Wait for Arduino pin to go high
	def waitForHigh(self):
		rospy.loginfo('Waiting for Arduino pin to go high...')
		signal.alarm(self.HS_TIMEOUT)
		while(self.readGPIO(self.ARDUINO_PIN_VAL) == '0'):
			pass
		signal.alarm(0)

	# Wait for Arduino pin to go low
	def waitForLow(self):
		rospy.loginfo('Waiting for Arduino pin to go low...')
		signal.alarm(self.HS_TIMEOUT)
		while(self.readGPIO(self.ARDUINO_PIN_VAL) == '1'):
			pass
		signal.alarm(0)

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
	try:
		ac = ArduinoComm()
	except rospy.ROSInterruptException:
		pass
