#!/usr/bin/python

# Subscribes to control messages received from the operator

import sys
import rospy

from bitstring import Bits, BitArray, BitStream
from std_msgs.msg import String

sys.path.append('~/catkin_ws/src/roborescue/src')
# ROS modules
from proto import ControlPacket, PacketError

class LightDriver():

	GPIO_ROOT = '/sys/class/gpio'

	LIGHT_PIN = 127 					# Arduino pin 26 / GPIO pin 127
	LIGHT_PIN_PATH = '{0}/gpio{1}'.format(GPIO_ROOT, LIGHT_PIN)
	LIGHT_PIN_DIR = '{0}/direction'.format(LIGHT_PIN_PATH)
	LIGHT_PIN_VAL = '{0}/value'.format(LIGHT_PIN_PATH)

	# Initialize ROS subscriber
	def __init__(self):
		self.light = False
		self.initpins()
		rospy.init_node('lights', anonymous=False)
		self.sub = rospy.Subscriber('control_msgs', String, self.handleControlPacket)
		rospy.spin()

	def handleControlPacket(self, data):
		packet = ControlPacket(data.data)
		rospy.loginfo(packet)
		if packet.lights == '1':
			rospy.loginfo('Toggling lights')
			self.light = not self.light
			if self.light:
				self.writeGPIO(self.LIGHT_PIN_VAL, '1')
			else:
				self.writeGPIO(self.LIGHT_PIN_VAL, '0')
			

	# Initialize the GPIO pin directions and values
	def initpins(self):
		if self.getGPIODirection(self.LIGHT_PIN_DIR) == 'in':
			self.setGPIODirection(self.LIGHT_PIN_DIR, 'out')
		self.writeGPIO(self.LIGHT_PIN_VAL, '0')

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

if __name__ == "__main__":
	try:
		ld = LightDriver()
	except rospy.ROSInterruptException:
		pass
