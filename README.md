# Fire-Suppression-Robot
![KakaoTalk_20220603_074348086](https://user-images.githubusercontent.com/86957779/174499474-52258ddf-489f-42f4-82da-b2a60a012ae8.jpg)

|구성요소|이름|설명|
|------|---|---|
|Main Controller|Jetson Xavier NX|영상처리, 라이다 데이터 처리, ROS|
|Lower Controller|Arduino MEGA 2560|모터 제어, 엔코더 데이터 수신 및 처리|
|2D LiDAR|Rplidar A1|SLAM, AMCL, Scan Matching 수행|
|USB CAM|Logitech USB Camera|화재 인식, 거리 추정|
|DC Encoder Motor|IG-42GM+Encoder 01TYPE(12v)|전륜 구동, 디퍼런셜 타입 로봇, 엔코더 데이터 송신|
|Battery|PowerCraft 25.6V 13,200mAh|제어기 전원공급, 모터 전원 공급|

# TF-Tree
![178145641-561c84d2-12e8-4a1d-a049-f69c97b96efb](https://user-images.githubusercontent.com/86957779/178148198-4110a3a8-0096-49ec-b073-37fc9d0b91d0.png)
