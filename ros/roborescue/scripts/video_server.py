#!/usr/bin/python

/dev/v4l/by-id/usb-046d_HD_Webcam_C525_36C31260-video-index0
gst-launch v4l2src device=/dev/v4l/by-id/usb-OmniVision_Technologies__Inc._USB_Camera-B3.04.06.1-video-index0  ! 'video/x-raw-yuv,width=320,height=240' ! ffmpegcolorspace ! jpegenc ! rtpjpegpay ! udpsink host=192.168.0.53 port=9000
gst-launch v4l2src device=/dev/v4l/by-id/usb-046d_HD_Webcam_C525_36C31260-video-index0 ! 'image/jpeg,width=320,height=240' ! rtpjpegpay ! udpsink host=192.168.0.53 port=9000
