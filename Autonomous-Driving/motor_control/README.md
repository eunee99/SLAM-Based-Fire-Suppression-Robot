# Motor Control
* controller: Arduino MEGA 2560

# 엔코더에 의해 측정된 RPM
* Current RPM = (60 * TICKS) / (t * PPR * 체배(2체배))

# cmd_vel 토픽을 RPM으로 계산하는 공식
* Target RPM = ((cmd_vel.linear.x * 60) - (cmd_vel.angular.z * 60 * ROBOT_WIDTH / 2) / (2 * pi * WHEEL_RADIUS))

* Target RPM과 Current RPM의 에러를 구하여 PI 제어 실행
![PID_Compensation_Animated](https://user-images.githubusercontent.com/86957779/178277915-38c423bd-7814-4fe4-ab4d-f1f5d12e2d44.gif)
