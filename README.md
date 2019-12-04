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

2. Camera angel Measure   
  1. opencv object detection  
  2. opencv contour && find mass center  
  3. calculate angel  
  
3. Measure Distance Code  
  //11.18(월) 

//11.21(목)  
1. 자동차  
  - jetbot detect하는 model 찾기  
  - 주행 오차 보정 방법 찾기  
  
2. Simulation  
  - 목적지 가는 알고리즘 짜고 simulation 할 방법 찾기  
  - robot은 목적지, 출발지, 전체 맵 크기, Potential 분포만 아는 상태  
  - 목적지로 가는 도중 obstacle/other robots detect하게 됨  
  - Mobile Robotics Simulation Toolbox (11.22(금) - 11.23(토))  
  - https://kr.mathworks.com/matlabcentral/fileexchange/66586-mobile-robotics-simulation-toolbox

## 12.4(수) 면담 요약  
  
1. 코드  
  >fun_dir = @(x) K_dir* sum((nanAngle1-atan((x(2)-pose1(2,1))/(x(1)-pose1(1,1)))).^2))   
  - nanAngle1에서 x와 맞지 않는 값이 항상 존재 -> 무엇을 고르든 fun_dir은 거의 변하지 않을것임  
   ->nanAngle1에서 하나를 고르거나 가까이 있는 angle값을 높이기 (알아서 NaN인 각도를 고를 것임)   
  >fun_obs = @(x) K_obs*sum(1-cdf(gm, ((x(1)-obstacle(:,1)).^2 + (x(2)-obstacle(:,2)).^2).^(1/2)))  
  - 가우시안 연산량이 많음   
  -> Bound function이나 발산하지 않는 분수 함수로 대체할 것  
  - 과도한 tool 사용 -> 부품 연동 불가
    
2.  알고리즘  
  - 로봇이 자신, 목적지의 절대 위치를 모르는 형태로 짜야함.  
    Potential function x / 목적지를 찾기 위해 obstacle이 없는 방향으로 조금씩 움직이고 sensor detection 할 것  
  - 카메라 보정  
    일정 거리만큼 움직인 후 lidar(camera)로 측정한 거리로 삼각형 그려서 각도 보정
    
