#!/usr/bin/python

import os
import rospy
import shlex
import signal
import subprocess
import sys

from proto import ControlPacket, PacketError
from std_msgs.msg import String

cam1pipe1 = ("gst-launch v4l2src "
				 "device=/dev/v4l/by-id/usb-046d_HD_Webcam_C525_36C31260-video-index0 "
				 "! 'image/jpeg,width=320,height=240' ! rtpjpegpay ! "
				 "udpsink host=192.168.0.53 port=9000")

cam2pipe1 = ("gst-launch v4l2src "
				 "device=/dev/v4l/by-id/usb-OmniVision_Technologies__Inc._USB_Camera-B3.04.06.1-video-index0 "
				 "! 'video/x-raw-yuv,width=320,height=240' ! ffmpegcolorspace ! jpegenc ! rtpjpegpay ! "
				 "udpsink host=192.168.0.53 port=9002")

DEVICE_ID_PATH = '/dev/v4l/by-id'
HWID_CAM1 = 'usb-046d_HD_Webcam_C525_36C31260-video-index0'
HWID_CAM2 = 'usb-OmniVision_Technologies__Inc._USB_Camera-B3.04.06.1-video-index0'
HWPATH_CAM1 = '{0}/{1}'.format(DEVICE_ID_PATH, HWID_CAM1)
HWPATH_CAM2 = '{0}/{1}'.format(DEVICE_ID_PATH, HWID_CAM2)

HOST = '192.168.0.53'				# operator station ip
PORT_CAM1 = 9000
PORT_CAM2 = 9002
	
def init_camera1():
	args = shlex.split(cam1pipe1)
	proc1 = subprocess.Popen(args, shell=False,
								  close_fds=True,
								  preexec_fn=os.setsid,)
		
def init_camera2():
	args = shlex.split(cam2pipe1)
	proc2 = subprocess.Popen(args, shell=False,
								  close_fds=True,
								  preexec_fn=os.setsid,)

def main(argv):
	if argv[0] == '1':
		print 'cam1 start'
		init_camera1()
	elif argv[0] == '2':
		init_camera2()
		
if __name__ == "__main__":
	try:
		main(sys.argv[1:])	
	except Exception:
		pass
