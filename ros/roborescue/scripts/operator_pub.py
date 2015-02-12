#!/usr/bin/python

# Publishes control messages received from the operator station

import rospy
import socket
import sys

from bitstring import BitArray, BitStream
from std_msgs.msg import String

PACKET_SIZE = 5				# Control message size in bytes
LISTEN_PORT = 6969			# Operator station control msgs
BUFFER_SIZE = 64

def publish():
	pub = rospy.Publisher('control_msgs', String, queue_size=20)
	rospy.init_node('operator_msgs', anonymous=False)
	rate = rospy.Rate(10)

	while not rospy.is_shutdown():
		# Create TCP/IP socket
		s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
		host = socket.gethostname()
		s.bind(('', LISTEN_PORT))
		s.listen(1)
		rospy.loginfo('Listening on port %s' %LISTEN_PORT)

		while True:
			conn, addr = s.accept()
			rospy.loginfo('Connection established with %s' %addr)
			try:
				rospy.loginfo('client connected: %s' %addr)
				while True:
					data = conn.recv(BUFFER_SIZE)
					rospy.loginfo('received "%s"' %data)
					pub.publish(data)
					parsebyte(data)
					if data:
						bytes_sent = conn.send(data)
						rospy.loginfo('sent "%s" bytes' %bytes_sent)
					else:
						break
			finally:
				conn.close()

def parsedata(data):
	ba = BitStream(bytes=data, length=PACKET_SIZE)	
	if(ba.bin[0] == '0'):
		rospy.loginfo('FORWARD SPEED: "%s"' %ba.bin[1:8])
	elif(ba.bin[0] == '1'):
		rospy.loginfo('BACKWARD SPEED: "%s"' %ba.bin[1:8])

	forward = ba[0]						# bit 0: move forward/backward
	speedmag = ba[1:8]					# bits 1-7: speed
	leftright = ba[8]					# bit 8: turn left/right
	#turnmag = ba[]
	#frontflipper = ba[]
	#backflipper = ba[]
	arm = ba[14:17]
	pan = ba[18:19]						# bits 18,19: pan
	tilt = ba[20:21]					# bits 20,21: tilt
	lights = ba[22]						# bit 22: lights on/off
	robot_shutdown = ba[3:5:8]			# 5th byte (robot shutdown)

if __name__ == "__main__":
	try:
		publish()
	except rospy.ROSInterruptException:
		pass
