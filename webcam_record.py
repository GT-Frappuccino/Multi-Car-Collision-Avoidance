import numpy as np
import cv2

FRAME_WIDTH = 320
FRAME_HEIGHT = 240

cv2.namedWindow('Measure Angle with centerline')

# WebCam Initialize
vidCapture = cv2.VideoCapture(1)

fourcc = cv2.VideoWriter_fourcc(*'XVID') 
out = cv2.VideoWriter('webcam_record.avi', fourcc, 20.0, (640, 480)) 

while True:

	# key = cv2.waitKey(1) & 0xFF
	# if key == 27:
	# 	break

	ret, frame = vidCapture.read()
	
	if ret==True:
		# frame = cv2.flip(frame,0)

        # write the flipped frame
		out.write(frame)

		cv2.imshow('frame',frame)
		if cv2.waitKey(1) & 0xFF == ord('q'):
			break
	else:
		break
	# img = np.zeros((512, 512, 3), np.uint8)
	# cv2.line(frame, (160, 0), (160, 240), (255, 0, 0), 2)
	# cv2.line(frame, (0, 120), (320, 120), (255, 0, 0), 2)

	# cv2.imshow('frame', frame)

vidCapture.release()
out.release()
cv2.destroyAllWindows()