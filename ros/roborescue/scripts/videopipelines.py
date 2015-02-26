#!/usr/bin/python

import multiprocessing
import os
import shlex
import signal
import subprocess
import sys

from proto import ControlPacket, PacketError
from std_msgs.msg import String

def worker(pipeline, event):
	procpid = multiprocessing.current_process().pid
	print 'Worker process: {0}'.format(procpid)
	args = shlex.split(pipeline)
	gstproc = subprocess.Popen(args, close_fds=True, preexec_fn=os.setsid,
								stdout=subprocess.PIPE, stderr=subprocess.PIPE)
	print 'GStreamer process: {0}'.format(gstproc.pid)
	event.wait()
	os.kill(gstproc.pid, signal.SIGTERM)
