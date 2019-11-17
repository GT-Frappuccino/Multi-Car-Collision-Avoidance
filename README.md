# Multi-Car-Collision-Avoidance

ShowResult_highlow.py

-lower range, higher range

`colorLow = np.array([lowHue,lowSat,lowVal])
`colorHigh = np.array([highHue,highSat,highVal])

-draw rectangle contour

>cv2.rectangle(frame,(x,y),(x+w,y+h),(0,255,0),2)

-Show final output image

>cv2.imshow('colorTest', frame)


## To do
*1. 카메라 1개로 각도 측정
  
