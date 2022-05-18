import time
import numpy as np
import cv2
import time
import queue


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
    FPS = 20

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

        # 출력 영상 크기 픽셀 단위로 조정
        frame = cv2.resize(frame, (640, 480))

        blur = cv2.GaussianBlur(frame, (5, 5), 0)  # 가우시안 필터
        hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)  # hsv 형식으로 변경

        lower = np.array([15, 150, 20])  # 노랑색 탐지하도록 범위 지정
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

                    new_x = int(avg_x+avg_w/2)
                    new_y = int(avg_y+avg_h/2)

                    #cv2.rectangle(frame, (x, y), (x+w, y+h),(0, 0, 255), 2)
                    cv2.rectangle(frame, (avg_x, avg_y), (avg_x+avg_w, avg_y+avg_h),
                                  (0, 0, 255), 2)
                    #cv2.line(frame, (avg_x-100, avg_y+h),(avg_x+avg_w+100, avg_y+avg_h), (0, 0, 255), 2)
                    #cv2.putText(frame, 'fire', (x, y-7), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
                    cv2.putText(frame, 'distance : ' + str((avg_w+avg_h)/2) + 'cm',
                                (avg_x, avg_y-7), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

                    #print("중심좌표 : " + str(new_x) + ", " + str(new_y))
                    #print("측정 거리 : " + str((avg_w+avg_h)/2))

        cv2.imshow('Output', mask)
        cv2.imshow('Original', frame)

        key = cv2.waitKey(10) & 0xFF  # ESC를 누르면 종료
        if (key == 27):
            break

    cap.release()
    cv2.destroyAllWindows()


showVideo()
