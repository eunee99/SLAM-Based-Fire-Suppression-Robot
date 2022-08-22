# Fire-Suppression-Robot
![image](https://user-images.githubusercontent.com/86957779/185987023-7babd4d3-3c5d-41f4-9ef1-4cdf10b76ff5.png)


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
* AMCL을 통한 Map 상에서의 현재 위치 추정, 지역 경로계획 수행

# TF-Tree
![178145641-561c84d2-12e8-4a1d-a049-f69c97b96efb](https://user-images.githubusercontent.com/86957779/178148198-4110a3a8-0096-49ec-b073-37fc9d0b91d0.png)

# 화재 인식
![image](https://user-images.githubusercontent.com/86957779/185987081-1866f0de-ed49-4cf7-a636-a30b4db73943.png)
![image](https://user-images.githubusercontent.com/86957779/185987090-5edf1bf3-42b8-470b-a40f-ab2cb9fad946.png)
* 화재 경보 발생, 서버에 화재 경보 전송

# 화재 진압
![image](https://user-images.githubusercontent.com/86957779/185987354-9cf9e327-e6ac-44ea-a1d0-fedbe4df4210.png)
![image](https://user-images.githubusercontent.com/86957779/185987380-957c65bd-0865-4e52-951e-ffb48a62180d.png)
![image](https://user-images.githubusercontent.com/86957779/185987402-8ce67b4a-5925-4242-ae99-0610632bd6aa.png)
* BGR to HSV 변환하여 화재를 인식하고 주위에 Bounding Box 생성, Bounding Box의 넓이에 따른 실제 거리 값을 직접 측정하여 데이터를 구상하고, 2차 비선형 모델 추출
* 화재 감지기가 설치되어 있는 미리 설정된 Waypoint로 이동
* 장전된 투척용 소화기 발사
