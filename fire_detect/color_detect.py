import numpy as np
import cv2
import time
import queue
import math

class MovAvgFilter:
    
    # 이전 스텝의 평균
    prevAvg = 0
    # 가장 최근 n개의 값을 저장하는 큐
    xBuf = queue.Queue()
    # 참조할 데이터의 갯수
    n = 0

    def __init__(self, _n):
        # 초기화로 n개의 값을 0으로 둡니다.
        for _ in range(_n):
            self.xBuf.put(0)
        # 참조할 데이터의 갯수를 저장합니다.
        self.n = _n

    def movAvgFilter(self, x):
        # 큐의 front 값은 x_(k-n) 에 해당합니다.
        front = self.xBuf.get()
        # 이번 스텝에 입력 받은 값을 큐에 넣습니다.
        self.xBuf.put(x)

        avg = self.prevAvg + (x - front) / self.n
        self.prevAvg = avg

        return avg


def showVideo():

    avg_filter_x = MovAvgFilter(30)
    avg_filter_y = MovAvgFilter(30)
    avg_filter_w = MovAvgFilter(30)
    avg_filter_h = MovAvgFilter(30)

    prev_time = 0
    FPS = 15

    try:
        #cap = cv2.VideoCapture(1)  # 0: default camera
        cap=cv2.VideoCapture(cv2.CAP_DSHOW+ 1)  #web cam
        print('Open camera')
    except:
        print('Not work')         #예외처리

    while True:
        ret, frame = cap.read()  # 카메라 프레임 읽기
        
        if not ret:              # 프레임을 읽기 실패
            print("Video error")
            break

        # FPS 조절
        current_time = time.time() - prev_time

        if (ret is True) and (current_time > 1. / FPS):
            prev_time = time.time()

        # 출력 영상 크기 픽셀 단위로 조정(가로x세로)
        frame = cv2.resize(frame, (640, 480))

        blur = cv2.GaussianBlur(frame, (5,5), 0)  # 가우시안 필터
        hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)  # hsv 형식으로 변경

        lower = np.array([0, 150, 100])  # 빨~노 탐지하도록 범위 지정
        upper = np.array([35, 255, 255])

        mask = cv2.inRange(hsv, lower, upper)

        contours, _ = cv2.findContours(  # 윤곽선 탐지, 바깥라인 꼭짓점만 반환
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if len(contours) != 0:
            for contour in contours:
                if cv2.contourArea(contour) > 3000:

                    x, y, w, h = cv2.boundingRect(contour)

                    avg_x = int(avg_filter_x.movAvgFilter(x))
                    avg_y = int(avg_filter_y.movAvgFilter(y))
                    avg_w = int(avg_filter_w.movAvgFilter(w))
                    avg_h = int(avg_filter_h.movAvgFilter(h))
                    
                    #box_core
                    box_x = int(avg_x+avg_w/2)  
                    box_y = int(avg_y+avg_h/2)

                    #cv2.rectangle(frame, (x, y), (x+w, y+h),(0, 0, 255), 2)
                    cv2.rectangle(frame, (avg_x, avg_y), (avg_x+avg_w, avg_y+avg_h),
                                  (0, 0, 255), 2)
                    #cv2.line(frame, (avg_x-100, avg_y+h),(avg_x+avg_w+100, avg_y+avg_h), (0, 0, 255), 2)
                    #cv2.putText(frame, 'fire', (x, y-7), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

                    x=[10,20,30,40,50,60]#박스크기
                    y=[70,60,50,40,30,20]#거리
                    fun=np.polyfit(x,y,1) #y = fun[0]*x + fun[1]
                    dia=math.sqrt((avg_w**2)+(avg_h**2))   #대각선길이
                    distance=fun[0]*dia+fun[1]

                    cv2.putText(frame, 'distance : ' + str(round(distance,2)) + 'cm',
                                (avg_x, avg_y-7), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

                    #print("중심좌표 : " + str(new_x) + ", " + str(new_y))
                    #print("측정 거리 : " + str((avg_w+avg_h)/2))
                    
                    #박스와 화면 중점간의 거리(좌표평면과 +/- 동일)
                    distance_x=box_x-320 #박스가 오른쪽에 있으면 +, 왼쪽에 있으면 -
                    distance_y=240-box_y #박스가 위쪽에 있으면 +, 아래쪽에 있으면 -
                    
                    #distance_core = int(math.sqrt(distance_x**2 + distance_y**2))
                    print("("+str(distance_x)+","+str(distance_y)+")")

        cv2.imshow('Output', mask)
        cv2.imshow('Original', frame)
        
        key = cv2.waitKey(10) & 0xFF  # ESC를 누르면 종료
        if (key == 27):
            break

    cap.release()
    cv2.destroyAllWindows()


showVideo()
