import numpy as np
import cv2
import time
import math

FRAME_WIDTH = 640
FRAME_HEIGHT = 480

FRAME_DISTANCE_WIDTH = FRAME_WIDTH/(2*math.tan(math.radians(26.6)))
FRAME_DISTANCE_HEIGHT = FRAME_HEIGHT/(2*math.tan(math.radians(19.0)))
# WebCam Initialize
# cap = cv2.VideoCapture('webcam_record.avi')
cap = cv2.VideoCapture(1)
cap.set(cv2.CAP_PROP_FRAME_WIDTH,FRAME_WIDTH)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT,FRAME_HEIGHT)

ret, frame1 = cap.read()
ret, frame2 = cap.read()

# def select_point(event, x, y, flage, params):
# 	if event == cv2.EVENT_LBUTTONDOWN:
# 		pass
# 	pass

while cap.isOpened():
	diff = cv2.absdiff(frame1, frame2)
	gray = cv2.cvtColor(diff, cv2.COLOR_BGR2GRAY)

	blur = cv2.GaussianBlur(gray, (5,5), 0)
	_, thresh = cv2.threshold(blur, 20, 255, cv2.THRESH_BINARY)
	dilated = cv2.dilate(thresh, None, iterations=4)
	_, contours, _ = cv2.findContours(dilated, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

	# contour_index = 0
	for contour in contours:

		# draw rectangle contour
		(x, y, w, h) = cv2.boundingRect(contour)
		contour_center = cv2.moments(contour)
		cX = int(contour_center["m10"] / contour_center["m00"])
		cY = int(contour_center["m01"] / contour_center["m00"])

		if cv2.contourArea(contour) < 700:
			continue

		# contour_index = contour_index + 1
		cv2.rectangle(frame1, (x, y), (x+w, y+h), (0, 255, 0), 2)
		cv2.circle(frame1, (cX, cY), 7, (255, 255, 255), -1)
		angleX = math.degrees(math.atan((cX-FRAME_WIDTH/2)/FRAME_DISTANCE_WIDTH))
		angleY = math.degrees(math.atan((FRAME_HEIGHT/2-cY)/FRAME_DISTANCE_HEIGHT))
		cv2.putText(frame1,"cX: {}".format(cX), (10, 20), cv2.FONT_HERSHEY_SIMPLEX,
			1, (0, 0, 255), 2)
		cv2.putText(frame1,"cY: {}".format(cY), (10, 50), cv2.FONT_HERSHEY_SIMPLEX,
			1, (0, 0, 255), 2)
		cv2.putText(frame1,"angleX: {}".format(angleX), (10, 80), cv2.FONT_HERSHEY_SIMPLEX,
			1, (0, 0, 255), 2)
		cv2.putText(frame1,"angleY: {}".format(angleY), (10, 110), cv2.FONT_HERSHEY_SIMPLEX,
			1, (0, 0, 255), 2)
		
	cv2.line(frame1, (int(FRAME_WIDTH/2), 0), (int(FRAME_WIDTH/2), FRAME_HEIGHT), (255, 0, 0), 2)
	cv2.line(frame1, (0, int(FRAME_HEIGHT/2)), (FRAME_WIDTH, int(FRAME_HEIGHT/2)), (255, 0, 0), 2)

	cv2.imshow('Tracking and Finding angle of moving object',frame1)
	frame1 = frame2
	ret, frame2 = cap.read() 

	key = cv2.waitKey(1) & 0xFF
	if key == 27:
		break

	time.sleep(0.1)

cap.release()
cv2.destroyAllWindows()