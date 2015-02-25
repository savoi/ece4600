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
		self.proc1 = None
		self.proc2 = None
		rospy.init_node('videoserver', anonymous=False)
		self.sub = rospy.Subscriber('control_msgs', String, self.handleControlPacket)
		rospy.spin()
	
	# Toggles cameras one and two on/off
	def handleControlPacket(self, data):
		packet = ControlPacket(data.data)
		if packet.camone == '11':
			if not self.proc1:			# server not running
				rospy.loginfo('Starting video server 1')	
				self.init_camera1()	
			else:						# server running
				rospy.loginfo('Killing video server 1')	
				os.kill(self.proc1.pid, signal.SIGTERM)
				self.proc1 = None
		if packet.camtwo == '11':
			if not self.proc2:			# server not running
				rospy.loginfo('Starting video server 2')	
				self.init_camera2()	
			else:						# server running
				rospy.loginfo('Killing video server 2')	
				os.kill(self.proc2.pid, signal.SIGTERM)
				self.proc2 = None
	
	def init_camera1(self):
		args = ["/usr/bin/python", "/home/ubuntu/catkin_ws/src/roborescue/scripts/videopipelines.py", "1"]
		self.proc1 = subprocess.Popen(args, shell=False,
									  close_fds=True,
									  preexec_fn=os.setsid,)
		
	def init_camera2(self):
		args = ["/usr/bin/python", "/home/ubuntu/catkin_ws/src/roborescue/scripts/videopipelines.py", "2"]
		self.proc2 = subprocess.Popen(args, shell=False,
									  close_fds=True,
									  preexec_fn=os.setsid,)
		
if __name__ == "__main__":
	try:
		vs = VideoServer()
	except rospy.ROSInterruptException:
		pass
