# Fire-Suppression-Robot
![image](https://user-images.githubusercontent.com/86957779/185987023-7babd4d3-3c5d-41f4-9ef1-4cdf10b76ff5.png)
![KakaoTalk_20220914_164038237_01](https://user-images.githubusercontent.com/86957779/190152195-c0194e4a-84b8-41a2-adce-a79c43bdf7d7.jpg)
> CP-CoP 금상 수상작

|구성요소|이름|설명|
|------|---|---|
|Main Controller|Jetson Xavier NX|영상처리, 라이다 데이터 처리, ROS|
|Lower Controller|Arduino MEGA 2560|모터 제어, 엔코더 데이터 수신 및 처리|
|2D LiDAR|Rplidar A1|SLAM, AMCL, Scan Matching 수행|
|USB CAM|Logitech USB Camera|화재 인식, 거리 추정|
|DC Encoder Motor|IG-42GM+Encoder 01TYPE(12v)|전륜 구동, 디퍼런셜 타입 로봇, 엔코더 데이터 송신|
|Battery|PowerCraft 25.6V 13,200mAh|제어기 전원공급, 모터 전원 공급|

# 시스템 구성도
![image](https://user-images.githubusercontent.com/86957779/185987694-03db2709-2f73-4166-aa3c-c3a3c595b857.png)

# SLAM
![image](https://user-images.githubusercontent.com/86957779/185987179-14475b7f-5956-483d-a58d-810789d5b1cb.png)
![image](https://user-images.githubusercontent.com/86957779/185987285-de8bdefd-3611-46f0-bb14-27bdf27525bc.png)
* Gmapping SLAM을 이용한 산학융합관 Map 제작
* AMCL을 통한 Map 상에서의 현재 위치 추정
* Dynamic Window Approach 기반의 지역 경로계획 수행

# TF-Tree
![178145641-561c84d2-12e8-4a1d-a049-f69c97b96efb](https://user-images.githubusercontent.com/86957779/178148198-4110a3a8-0096-49ec-b073-37fc9d0b91d0.png)

# 화재 진압
![image](https://user-images.githubusercontent.com/86957779/185987354-9cf9e327-e6ac-44ea-a1d0-fedbe4df4210.png)
![image](https://user-images.githubusercontent.com/86957779/185987380-957c65bd-0865-4e52-951e-ffb48a62180d.png)
![image](https://user-images.githubusercontent.com/86957779/185987402-8ce67b4a-5925-4242-ae99-0610632bd6aa.png)
* BGR to HSV 변환하여 화재를 인식하고 주위에 Bounding Box 생성, Bounding Box의 넓이에 따른 실제 거리 값을 직접 측정하여 데이터를 구상하고, 2차 비선형 모델 추출
* 화재 감지기가 설치되어 있는 미리 설정된 Waypoint로 이동
* 상단부에 부착된 Servo Motor로 스프링에 장전된 투척용 소화기를 발사

# Motor Control
* controller: Arduino MEGA 2560

## 엔코더에 의해 측정된 RPM
* Current RPM = (60 * TICKS) / (t * PPR * 체배(2체배))

## cmd_vel 토픽을 RPM으로 계산하는 공식
* Target RPM = ((cmd_vel.linear.x * 60) - (cmd_vel.angular.z * 60 * ROBOT_WIDTH / 2) / (2 * pi * WHEEL_RADIUS))

* Target RPM과 Current RPM의 에러를 구하여 PI 제어 실행  
![PID_Compensation_Animated](https://user-images.githubusercontent.com/86957779/178277915-38c423bd-7814-4fe4-ab4d-f1f5d12e2d44.gif)
