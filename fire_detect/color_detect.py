#!/usr/bin/env python
# -*- coding: utf-8 -*-

import time
import numpy as np
import cv2
import queue 

distance_data = np.array([50, 55, 60, 65, 70, 75, 80, 85, 90, 95, 97, 100, 103, 105, 109, 120])
extent_data = np.array([110558, 89780, 90468, 77356, 64752, 56516, 49929, 43750, 38940, 34788, 33454, 30870, 29172, 28000, 25728, 20230])
expression = np.polyfit(extent_data, distance_data, deg=2)
expression = np.poly1d(expression)

class MovAvgFilter:
    # 이전 스텝의 평균
    prevAvg = 0
    # 가장 최근 n개의 값을 저장하는 큐
    xBuf = queue.Queue()
    # 참조할 데이터의 갯수
    n = 0
 
    def __init__(self, _n):
        for _ in range(_n):
            self.xBuf.put(0)
        self.n = _n
 
    def movAvgFilter(self, x):
        front = self.xBuf.get()
        self.xBuf.put(x)
 
        avg = self.prevAvg + (x - front) / self.n
        self.prevAvg = avg
 
        return avg
 
def showVideo():
    avg_filter_x = MovAvgFilter(10)
    avg_filter_y = MovAvgFilter(10)
    avg_filter_w = MovAvgFilter(10)
    avg_filter_h = MovAvgFilter(10)
 
    prev_time = 0
    FPS = 15
 
    try:
        cap = cv2.VideoCapture(0)  # 0: default camera
        print('Open camera')
    except:
        print('Not work')
 
    while True:
        ret, frame = cap.read()  # 카메라 프레임 읽기
        if not ret:
            print("Video error")
            break
 
        # FPS 조절
        current_time = time.time() - prev_time
 
        if (ret is True) and (current_time > 1. / FPS):
            prev_time = time.time()
 
        frame = cv2.resize(frame, (640, 480))
 
        # 프레임 중심좌표
        x, y, z = frame.shape
        mid_x = int(x/2)
        mid_y = int(y/2)
        mid_coor = np.array([mid_x, mid_y])
 
        blur = cv2.GaussianBlur(frame, (5, 5), 0)  # 가우시안 필터
        hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)  # hsv 형식으로 변경
 
        lower = np.array([0, 150, 100])  # 빨~노 탐지하도록 범위 지정
        upper = np.array([35, 255, 255])
 
        mask = cv2.inRange(hsv, lower, upper)
 
        _, contours, _ = cv2.findContours(  # 윤곽선 탐지, 바깥라인 꼭짓점만 반환
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
 
        if len(contours) != 0:
            for contour in contours:
                if cv2.contourArea(contour) > 3000:
 
                    x, y, w, h = cv2.boundingRect(contour)
 
                    avg_x = int(avg_filter_x.movAvgFilter(x))
                    avg_y = int(avg_filter_y.movAvgFilter(y))
                    avg_w = int(avg_filter_w.movAvgFilter(w))
                    avg_h = int(avg_filter_h.movAvgFilter(h))
 
                    # bounding box 중심좌표
                    new_x = int(avg_x+avg_w/2)
                    new_y = int(avg_y+avg_h/2)
                    new_coor = np.array([new_x, new_y])
                    print(new_coor)

                    extent = avg_w * avg_h
                    distance =np.round(expression(extent), 1)
                    cv2.rectangle(frame, (avg_x, avg_y), (avg_x+avg_w, avg_y+avg_h),
                                  (0, 0, 255), 2)

                    cv2.putText(frame, 'distance : ' + str(distance) + 'cm', (avg_x, avg_y-7), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

        cv2.imshow('Original', frame)
 
        key = cv2.waitKey(10) & 0xFF  # ESC를 누르면 종료
        if (key == 27):
            break
 
    cap.release()
    cv2.destroyAllWindows()
 
showVideo()
