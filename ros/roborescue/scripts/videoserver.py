#!/usr/bin/python

import multiprocessing as mp
import os
import rospy
import signal
import sys
import time
import videopipelines

from bitstring import BitArray
from proto import ControlPacket, PacketError
from std_msgs.msg import String

class VideoServer:

	DEVICE_ID_PATH = '/dev/v4l/by-id'
	HWID_CAM1 = 'usb-046d_HD_Webcam_C525_36C31260-video-index0'
	HWID_CAM2 = 'usb-OmniVision_Technologies__Inc._USB_Camera-B3.04.06.1-video-index0'
	HWPATH_CAM1 = '{0}/{1}'.format(DEVICE_ID_PATH, HWID_CAM1)
	HWPATH_CAM2 = '{0}/{1}'.format(DEVICE_ID_PATH, HWID_CAM2)
	HOST = '192.168.0.53'				# operator station ip
	PORT_CAM1 = 9000
	PORT_CAM2 = 9002

	cam1pipe1_attrs = (HWPATH_CAM1, HOST, PORT_CAM1)
	cam1pipe1 = ("gst-launch v4l2src device={0} ! 'image/jpeg,width=320,height=240' ! "
				 "rtpjpegpay ! udpsink host={1} port={2}".format(HWPATH_CAM1, HOST, PORT_CAM1))

	cam2pipe1 = ("gst-launch v4l2src "
				 "device={0} ! videorate "
				 "! 'video/x-raw-yuv,width=320,height=240,framerate=30/1' ! ffmpegcolorspace ! "
				 "jpegenc ! rtpjpegpay ! "
				 "udpsink host=192.168.0.53 port=9002".format(HWPATH_CAM2))

	# Initialize ROS subscriber
	def __init__(self):
		rospy.init_node('videoserver', anonymous=False)
		rospy.loginfo('Started videoserver node')
		self.pub = rospy.Publisher('status_msgs', String)
		self.status = BitArray('0b11111111000000100000000000000000')
		self.pluggedin = [True, True]
		self.event1 = mp.Event()
		self.event2 = mp.Event()
		self.workerinit_1()
		self.workerinit_2()
		self.sub = rospy.Subscriber('control_msgs', String, self.handleControlPacket)
		rospy.on_shutdown(self.shutdownhook)
		rospy.loginfo('Waiting for operator controls...')
		rospy.spin()

	# Handles node shutdown when initiated by operator
	def shutdownhook(self):
		self.sub.unregister()
		self.pub.unregister()
		self.killall()
		rospy.logwarn('!!! VIDEO SERVER SHUTDOWN !!!')
	
	# Toggles cameras one and two on/off
	def handleControlPacket(self, data):
		if rospy.has_param('/operator_shutdown'):
			if rospy.get_param('/operator_shutdown') == True:
				signal_shutdown('!!! OPERATOR INITIATED SHUTDOWN !!!')
		try:
			packet = ControlPacket(data.data)
			if packet.camone == '11':
				if self.pluggedin[0]:
					# video server 1 not running
					if not self.vidworker_1.is_alive():
						rospy.loginfo('Starting video server 1')	
						self.vidworker_1.start()
						self.status[23] = 1
					# kill video server 1
					else:
						self.killcam1()
				else:
					self.workerinit_1()	
			if packet.camtwo == '11':
				if self.pluggedin[1]:
					# video server 2 not running
					if not self.vidworker_2.is_alive():
						rospy.loginfo('Starting video server 2')	
						self.vidworker_2.start()
						self.status[22] = 1
					# kill video server 2
					else:
						self.killcam2()
				else:
					self.workerinit_2()
			self.pub.publish(self.status.bytes)
		except PacketError:
			rospy.logwarn('Packet parsing error.')

	# Kill all the camera processes
	def killall(self):
		self.killcam1()
		self.killcam2()

	# Kills the camera 1 process and pipeline subprocess
	def killcam1(self):
		rospy.loginfo('Killing video server 1')	
		self.event1.set()
		time.sleep(3)
		self.event1.clear()
		self.workerinit_1()
		self.status[23] = 0

	# Kills the camera 2 process and pipeline subprocess
	def killcam2(self):
		rospy.loginfo('Killing video server 2')	
		self.event2.set()
		time.sleep(3)
		self.event2.clear()
		self.workerinit_2()
		self.status[22] = 0

	# Test whether a camera is plugged in
	def ispluggedin(self, path):
		if os.path.exists(path):
			return True
		else:
			return False

	# Initialize first camera process template and event signal
	def workerinit_1(self):
		if self.ispluggedin(self.HWPATH_CAM1):
			self.pluggedin[0] = True
			self.status[21] = 1
			self.event1 = mp.Event()
			self.vidworker_1 = mp.Process(name='vidworker_1',
									  	  target=videopipelines.worker,
									  	  args=(self.cam1pipe1, self.event1))
		else:
			rospy.logwarn('!!! Camera 1 is not plugged in !!!')
			self.pluggedin[0] = False
			self.status[21] = 0
			self.status[23] = 0
		self.pub.publish(self.status.bytes)

	# Initialize second camera process template and event signal
	def workerinit_2(self):
		if self.ispluggedin(self.HWPATH_CAM2):
			self.pluggedin[1] = True
			self.status[20] = 1
			self.event2 = mp.Event()
			self.vidworker_2 = mp.Process(name='vidworker_2',
									  	  target=videopipelines.worker,
									  	  args=(self.cam2pipe1, self.event2))
		else:
			rospy.logwarn('!!! Camera 2 is not plugged in !!!')
			self.pluggedin[1] = False
			self.status[20] = 0
			self.status[22] = 0
		self.pub.publish(self.status.bytes)
	
if __name__ == "__main__":
	try:
		vs = VideoServer()
	except rospy.ROSInterruptException:
		pass
