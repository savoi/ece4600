#!/usr/bin/python

import multiprocessing as mp
import os
import rospy
import signal
import time
import videopipelines

from proto import ControlPacket, PacketError
from std_msgs.msg import String

class VideoServer:

	cam1pipe1 = ("gst-launch v4l2src "
				 "device=/dev/v4l/by-id/usb-046d_HD_Webcam_C525_36C31260-video-index0 "
				 "! 'image/jpeg,width=320,height=240' ! rtpjpegpay ! "
				 "udpsink host=192.168.0.53 port=9000")

	cam2pipe1 = ("gst-launch v4l2src "
				 "device=/dev/v4l/by-id/usb-OmniVision_Technologies__Inc._USB_Camera-B3.04.06.1-video-index0 "
				 "! 'video/x-raw-yuv,width=320,height=240' ! ffmpegcolorspace ! "
				 "jpegenc ! rtpjpegpay ! "
				 "udpsink host=192.168.0.53 port=9002")

	DEVICE_ID_PATH = '/dev/v4l/by-id'
	HWID_CAM1 = 'usb-046d_HD_Webcam_C525_36C31260-video-index0'
	HWID_CAM2 = 'usb-OmniVision_Technologies__Inc._USB_Camera-B3.04.06.1-video-index0'
	HWPATH_CAM1 = '{0}/{1}'.format(DEVICE_ID_PATH, HWID_CAM1)
	HWPATH_CAM2 = '{0}/{1}'.format(DEVICE_ID_PATH, HWID_CAM2)

	HOST = '192.168.0.53'				# operator station ip
	PORT_CAM1 = 9000
	PORT_CAM2 = 9002

	# Initialize ROS subscriber
	def __init__(self):
		self.workerinit_1()
		self.workerinit_2()
		rospy.init_node('videoserver', anonymous=False)
		rospy.loginfo('Started videoserver node')
		self.sub = rospy.Subscriber('control_msgs', String, self.handleControlPacket)
		rospy.loginfo('Waiting for operator controls...')
		rospy.spin()
	
	# Toggles cameras one and two on/off
	def handleControlPacket(self, data):
		try:
			packet = ControlPacket(data.data)
			if packet.camone == '11':
				# video server 1 not running
				if not self.vidworker_1.is_alive():
					rospy.loginfo('Starting video server 1')	
					self.vidworker_1.start()
				# kill video server 1
				else:
					rospy.loginfo('Killing video server 1')	
					self.event1.set()
					time.sleep(3)
					rospy.loginfo('{0} is alive: {1}'.format(self.vidworker_1.name, self.vidworker_1.is_alive()))	
					self.event1.clear()
					self.workerinit_1()
			if packet.camtwo == '11':
				# video server 2 not running
				if not self.vidworker_2.is_alive():
					rospy.loginfo('Starting video server 2')	
					self.vidworker_2.start()
				# kill video server 2
				else:
					rospy.loginfo('Killing video server 2')	
					self.event2.set()
					time.sleep(3)
					rospy.loginfo('{0} is alive: {1}'.format(self.vidworker_2.name, self.vidworker_2.is_alive()))	
					self.event2.clear()
					self.workerinit_2()
		except PacketError:
			rospy.logwarn('Packet parsing error.')

	# Initialize first camera process template and event signal
	def workerinit_1(self):
		self.event1 = mp.Event()
		self.vidworker_1 = mp.Process(name='vidworker_1',
									  target=videopipelines.worker,
									  args=(self.cam1pipe1, self.event1))

	# Initialize second camera process template and event signal
	def workerinit_2(self):
		self.event2 = mp.Event()
		self.vidworker_2 = mp.Process(name='vidworker_2',
									  target=videopipelines.worker,
									  args=(self.cam2pipe1, self.event2))
	
if __name__ == "__main__":
	try:
		vs = VideoServer()
	except rospy.ROSInterruptException:
		pass
