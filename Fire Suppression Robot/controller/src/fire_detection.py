#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import Int8

class fire_detection:
    def __init__(self):
        self.distance_data = np.array([50, 55, 60, 65, 70, 75, 80, 85, 90, 95, 97, 100, 103, 105, 109, 120])
        self.extent_data = np.array([110558, 89780, 90468, 77356, 64752, 56516, 49929, 43750, 38940, 34788, 33454, 30870, 29172, 28000, 25728, 20230])
        
        self.expression = np.polyfit(self.extent_data, self.distance_data, deg=2)
        self.expression = np.poly1d(self.expression)
        
        self.lower = np.array([0, 150, 100])  # 빨~노 탐지하도록 범위 지정
        self.upper = np.array([35, 255, 255])
        self.kernel = np.ones((5, 5))
        self.dist_list = [53] # 최소 인지거리 53cm
        self.limit_dist = lambda x: 53.0 if (x <=53.0) else x
        self.n = 50

        self.ros_init()

    def ros_init(self):
        rospy.init_node('fire_detection')
        rospy.loginfo(rospy.get_name() + ' start !')
        rospy.Subscriber('image_raw', Image, self.showVideo)
	    self.motor_publisher = rospy.Publisher('fire_cmd_vel', Twist, queue_size = 10)
        self.motor_val = Twist()
        
        self.launcher_publisher = rospy.Publisher('launch_flag', Int8, queue_size = 10)
        self.launch_flag = 0
        self.bridge = CvBridge()
        self.rate = rospy.Rate(10)

    def MovingAverageFilter(self, x):
        if len(self.dist_list) >= self.n:
            self.dist_list.pop(0)
        self.dist_list.append(x)
        res = sum(self.dist_list) / len(self.dist_list)
        return np.round(res, 1)
    
    def calc_distance(self, extent):
        dist = np.round(self.expression(extent), 1)
        return dist

    def showVideo(self, data):
        frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
        frame = cv2.resize(frame, (640, 480))

        blur = cv2.GaussianBlur(frame, (5, 5), 0)  # 가우시안 필터
        hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)  # hsv 형식으로 변경
        morph_close = cv2.morphologyEx(hsv, cv2.MORPH_CLOSE, self.kernel)

        mask = cv2.inRange(morph_close, self.lower, self.upper)

        _, contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if len(contours) != 0:
            for contour in contours:
                if cv2.contourArea(contour) > 4500:
                    [x, y, w, h] = cv2.boundingRect(contour)

                    new_x = int(x + w/2) # bounding box 중심좌표
                    new_y = int(y + h/2) # bounding box 중심좌표

                    extent = w * h # bounding box 넓이
                    distance = self.calc_distance(extent)
                    distance = float(self.MovingAverageFilter(distance))
                    distance = self.limit_dist(distance)
                    
                    cv2.rectangle(frame, (x, y), (x+w, y+h),(0, 0, 255), 2)

                    cv2.putText(frame, 'distance : ' + str(distance) + 'cm', (x, y-7), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
                    
                    # calc angle value
                    core_disx = round(float((new_x - 320) / 320.0),2)

                    # motor control
                    self.motor_val.linear.x = round(0.1 / 95.0  * (distance - 95.0), 2)
                    self.motor_val.angular.z = core_disx * -0.5
            
                    print('linear x: {}, angular z: {}'.format(self.motor_val.linear.x, self.motor_val.angular.z))
                    
                    if (self.motor_val.angular.z <= 0.1) and (self.motor_val.linear.x <= 0.1):
                        self.launch_flag = 1
                        
                    if self.launch_flag == 1:
                        self.motor_val.linear.x = 0.0
                        self.motor_val.angular.z = 0.0
                        self.launcher_publisher.publish(1)

                    self.motor_publisher.publish(self.motor_val)
                    self.rate.sleep()

        cv2.imshow('fire', frame)
        cv2.waitKey(1)

if __name__=='__main__':
    fire = fire_detection()
    rospy.spin()
