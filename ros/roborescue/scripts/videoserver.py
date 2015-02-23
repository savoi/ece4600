#!/usr/bin/python

import os
import rospy
import shlex
import signal
import subprocess

from proto import ControlPacket, PacketError
from std_msgs.msg import String

class VideoServer:

	cam1pipe1 = ("gst-launch v4l2src "
				 "device=/dev/v4l/by-id/usb-046d_HD_Webcam_C525_36C31260-video-index0 "
				 "! 'image/jpeg,width=320,height=240' ! rtpjpegpay ! "
				 "udpsink host=192.168.0.53 port=9000")

	cam2pipe1 = ("gst-launch v4l2src "
				 "device=/dev/v4l/by-id/usb-046d_HD_Webcam_C525_36C31260-video-index0 "
				 "! 'image/jpeg,width=320,height=240' ! rtpjpegpay ! "
				 "udpsink host=192.168.0.53 port=9000")

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
		rospy.init_node('videoserver', anonymous=False)
		self.sub = rospy.Subscriber('control_msgs', String, self.handleControlPacket)
		# start video server
		self.init_camera1()
		#self.init_camera2()
	
	# Toggles cameras one and two on/off
	def handleControlPacket(self, data):
		packet = ControlPacket(data.data)
		if packet.camone == '11':
			if not self.proc1:			# server not running
				self.init_camera1()	
			else:						# server running
				self.proc1.terminate()
		if packet.camtwo == '11':
			if self.pid2 == -1:		# server not running
				self.init_camera2()	
			else:						# server running
				os.kill(self.pid2, signal.SIGHUP)
				self.pid2 = -1
	
	def init_camera1(self):
		args = shlex.split(self.cam1pipe1)
		self.proc1 = subprocess.Popen(args, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
		outval, errval = self.proc1.communicate()
		print errval
		
	def init_camera2(self):
		args = shlex.split(self.cam1pipe1)
		self.p2 = subprocess.Popen(args)
		
if __name__ == "__main__":
	try:
		vs = VideoServer()
	except rospy.ROSInterruptException:
		pass
