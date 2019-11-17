import numpy as np
import cv2
from picamera.array import PiRGBArray
from picamera import PiCamera
import time

# initialize the camera and grab a reference to the raw camera capture
camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 32
rawCapture = PiRGBArray(camera, size=(640, 480))

# allow the camera to warmup
time.sleep(0.1)

# capture frames from the camera
for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
	# grab the raw NumPy array representing the image, then initialize the timestamp
	# and occupied/unoccupied text
	image = frame.array

	# draw centerline
 	cv2.line(image, (320, 0), (320, 480), (255, 0, 0), 2)
	cv2.line(image, (0, 240), (640, 240), (255, 0, 0), 2)

	# show the frame
	cv2.imshow("Frame", image)
	key = cv2.waitKey(1) & 0xFF
 
	# clear the stream in preparation for the next frame
	rawCapture.truncate(0)
 
 	#if the 'p' key was pressed, save img	 
	if key == ord("p"):
		cv2.imwrite("Measure_angle_raspicam.jpg", image);

	# if the `q` key was pressed, break from the loop
	if key == ord("q"):
		break
