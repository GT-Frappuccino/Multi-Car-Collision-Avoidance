# Multi-Car-Collision-Avoidance


## To do

1. 카메라 1개로 각도 측정 // 일단은 가로만  
  1. 카메라 광각(?=theta)(가로, 세로)/2 측정
  >theta_raspicam = 64.6'  
  >theta_raspicam/2 = 32.3'
  
  >theta_webcam = 53.1' (width)  
  >theta_webcam/2 = 26.6'
  
  >theta_webcam = 37.9' (height)  
  >theta_webcam/2 = 19.0'
  
  2. img frame pixel(가로, 세로)/2 
  >raspicam_pixel = (640,480)  
  >raspicam_pixel_width/2 = 640/2 = 320 px
  
  >webcam_pixel = (320,240)  
  >webcam_pixel_width/2 = 320/2 = 160 px  
  >webcam_pixel_height/2 = 240/2 = 120 px
  
  3. focus to img frame distance = (img frame pixel/2)/tan(theta/2) 계산
  >focus to img frame distance_raspicam = 320/tan(32.3') = 506.7 (width)
  
  >focus to img frame distance_webcam = 160/tan(26.6') = 320 (width)  
  >focus to img frame distance_webcam = 120/tan(19.0') = 349 (height)  

2. Multi-Cam Board Test  

  1. 카메라 2대 연속촬영 delay test  
  2. short delay -> measure distance code  
       long delay -> 카메라 2대 동시 사용 방법 찾기
       
       webcam + raspicam 2대 사용하기
       
3. Measure Distance Code  
  //11.18(월) 

