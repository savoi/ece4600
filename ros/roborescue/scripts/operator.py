#!/usr/bin/python

# Publishes control messages received from the operator station
# Subscribes to status messages received from the arduino

import rospy
import select
import socket
import sys

from bitstring import Bits, BitArray, BitStream
from proto import ControlPacket, PacketError
from std_msgs.msg import String

class OperatorComm():

	PACKET_SIZE = 5				# Control message size in bytes
	LISTEN_PORT = 6969			# Operator station control msgs
	BUFFER_SIZE = 5
	
	connections = []		# list of socket clients
	pub = None				# ros publisher
	sub = None				# ros subscriber
	sock = None				# socket 
	conn = None				# client connection
	outgoing_msgs = [None]
	w_sockets = []
	r_sockets = []
	err_sockets = []

	# Initialize ROS publisher, subscriber and socket connection
	def __init__(self):
		rospy.init_node('operator', anonymous=False)
		self.rate = rospy.Rate(10)
		# Initialize ROS pub and sub
		self.pub = rospy.Publisher('control_msgs', String, queue_size=20)
		self.sub = rospy.Subscriber('status_msgs', String, self.handleArduinoData)
		rate = rospy.Rate(10)
		self.initSocket()
		
	# Create TCP/IP socket
	def initSocket(self):
		self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
		host = socket.gethostname()
		self.sock.bind(('', self.LISTEN_PORT))
		self.sock.listen(1)
		self.connections.append(self.sock)
		rospy.loginfo('Server listening on port {0}'.format(self.LISTEN_PORT))

	# Handler for sending status data to operator station from arduino
	def handleArduinoData(self, data):
		self.outgoing_msgs[0] = data.data

	def startNode(self):
		while not rospy.is_shutdown():
			self.r_sockets, self.w_sockets, self.err_sockets = select.select(self.connections, [], [])

			for sock in self.w_sockets:
				try:
					sock.send("HELLOE")
				except:
					rospy.logwarn('ERROR sending data')
		
			for sock in self.r_sockets:
				# new connection
				if sock == self.sock:
					conn, addr = sock.accept()
					self.connections.append(conn)
					rospy.loginfo('Client connected: {0}'.format(addr))
				# data incoming
				else:
					try:
						data = sock.recv(self.BUFFER_SIZE)
						if data:
							rospy.logdebug('received "{0}"'.format(data))
							try:
								packet = ControlPacket(data)
								if packet.robotshutdown == '11':
									rospy.set_param('operator_shutdown', True)
							except:
								pass
							self.pub.publish(data)
							bytes_sent = sock.send(str(self.outgoing_msgs[0]))
							rospy.logdebug('{0} bytes sent'.format(bytes_sent))
					except:
						rospy.loginfo('Client disconnected')
						sock.close()
						self.connections.remove(sock)
						continue
			self.rate.sleep()

if __name__ == "__main__":
	try:
		oc = OperatorComm()
		oc.startNode()
	except rospy.ROSInterruptException:
		pass
