#!/usr/bin/python

# Publishes control messages received from the operator station
# Subscribes to status messages received from the arduino

import rospy
import socket
import sys

from bitstring import Bits, BitArray, BitStream
from std_msgs.msg import String

class OperatorComm():

	PACKET_SIZE = 5				# Control message size in bytes
	LISTEN_PORT = 6969			# Operator station control msgs
	BUFFER_SIZE = 64

	pub = None				# ros publisher
	sub = None				# ros subscriber
	sock = None				# socket 
	conn = None				# client connection

	# Initialize ROS publisher, subscriber and socket connection
	def __init__(self):
		rospy.init_node('operator', anonymous=False)
		# Create TCP/IP socket
		self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
		host = socket.gethostname()
		self.sock.bind(('', self.LISTEN_PORT))
		self.sock.listen(1)
		rospy.loginfo('Listening on port %s' %self.LISTEN_PORT)
		self.conn, addr = self.sock.accept()
		rospy.loginfo('Connection established with {0}'.format(addr))

		# Initialize ROS pub and sub
		self.pub = rospy.Publisher('control_msgs', String, queue_size=20)
		self.sub = rospy.Subscriber('status_msgs', String, self.handleArduinoData)
		rate = rospy.Rate(10)

	# Handler for sending status data to operator station from arduino
	def handleArduinoData(self, data):
		bytes_sent = self.conn.send(str(data))
		rospy.loginfo('sent "{0}" bytes'.format(bytes_sent))

	def startNode(self):
		try:	
			while not rospy.is_shutdown():
				data = self.conn.recv(self.BUFFER_SIZE)
				if data:
					rospy.loginfo('received "{0}"'.format(data))
					self.pub.publish(data)
					self.parsedata(data)
		finally:
			self.conn.close()

	def parsedata(self, data):
		ba = Bits(bytes=data)	
		rospy.loginfo('{} bits received'.format(len(ba.bin)))
		if(ba.bin[0] == '0'):
			rospy.loginfo('FORWARD SPEED: "{0}"'.format(ba.bin[1:8]))
		elif(ba.bin[0] == '1'):
			rospy.loginfo('BACKWARD SPEED: "{0}"'.format(ba.bin[1:8]))

		forward = ba.bin[0]						# bit 0: move forward/backward
		speedmag = ba.bin[1:8]					# bits 1-7: speed
		leftright = ba.bin[8]					# bit 8: turn left/right
		if(leftright == '0'):
			rospy.loginfo('LEFT')
		else:
			rospy.loginfo('RIGHT')
		#turnmag = ba.bin[]
		#frontflipper = ba.bin[]
		#backflipper = ba.bin[]
		arm = ba.bin[14:17]
		pan = ba.bin[18:19]						# bits 18,19: pan
		tilt = ba.bin[20:21]					# bits 20,21: tilt
		lights = ba.bin[22]						# bit 22: lights on/off
		robot_shutdown = ba.bin[3:5:8]			# 5th byte (robot shutdown)

if __name__ == "__main__":
	try:
		oc = OperatorComm()
		oc.startNode()
	except rospy.ROSInterruptException:
		pass
