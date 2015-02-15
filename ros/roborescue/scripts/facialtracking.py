#!/usr/bin/python

# Test facial recognition using OpenCV

import cv2
import sys

cascPath = sys.argv[1]
faceCascade = cv2.CascadeClassifier(cascPath)

# Set video source
video_capture = cv2.VideoCapture(3)
video_capture.set(3,320)
video_capture.set(4,240)

while True:
	# Capture frame-by-frame
	ret, frame = video_capture.read()

	gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

	faces = faceCascade.detectMultiScale(
		gray,
		scaleFactor=1.2,
		minNeighbors=5,
		minSize=(30,30),
		flags=cv2.cv.CV_HAAR_SCALE_IMAGE
	)

	# Draw rectangle around faces
	for (x, y, w, h) in faces:
		cv2.rectangle(frame, (x,y), (x+w,y+h), (0,255,0), 2)
		
	# Display resulting frame
	cv2.imshow('Video', frame)

	if cv2.waitKey(1) and 0xFF == ord('q'):
		break

# When done, release capture
video_capture.release()
cv2.destroyAllWindows()
