#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import cv2, random, math, time, threading
from collections import deque

Width = 640
Height = 480
Offset = 315

# left_roi
xl_1, yl_1 = 90, 340
xl_2, yl_2 = 230, 340
xl_3, yl_3 = 10, 385
xl_4, yl_4 = 130, 400

# right_roi
xr_1, yr_1 = 420, 340
xr_2, yr_2 = 580, 320
xr_3, yr_3 = 535, 400
xr_4, yr_4 = 640, 375

prev_lines_l = [[[62, 368, 143, 313]]]
lines_l = [[[]]]
prev_lines_r = [[[560, 327, 617, 359]]]
lines_r = [[[]]]
left_grad = -1 * 0.63
right_grad = 0.63

steer_angle = 0


# draw rectangle
def draw_rectangle(img, x, y):

    cv2.rectangle(img, (x - 5, y-5),
                       (x + 5, y+5),
                       (0, 255, 0), 2)
    return img

def region_of_interest(img, vertices, color):
    mask = np.zeros_like(img)

    cv2.fillPoly(mask, vertices, color)
    #cv2.imshow('mask',mask)

    ROI_image = cv2.bitwise_and(img, mask)

    return ROI_image

def get_gradient(line):
    gradient = 0

    x1, y1, x2, y2 = line[0]

    if (x1 != x2): # zero division
        gradient = float(y2-y1)/float(x2-x1)

    return gradient

def process_lines(lines, dir):
    real_lines = []
    ##print("here",lines)
    if not (lines is None):
        prev_line = lines[0]
        
	for line in lines:
            x1, y1, x2, y2 = line[0]
            cur_gradient = get_gradient(line)
            prev_gradient = get_gradient(prev_line)
            #print("cur:",cur_gradient)
            #print("prev:",prev_gradient)

            if abs(cur_gradient - prev_gradient) < 0.1: # 기울기 차이로 튀는 직선 제거
                if abs(line[0][2] - prev_line[0][2]) < 20: # 좌표 차이로 튀는 직선 제거
                    #print("cur_point:", line[0][0])
                    #print("prev_point:",prev_line[0][0])

                    if abs(cur_gradient) > 0.12 and abs(cur_gradient) < 0.98: # 수평선, 수직선 제거
	                print(cur_gradient)
                        real_lines.append(line)
                        prev_line = line
                    else:
                        real_lines.append(prev_line)
                else:
                    real_lines.append(prev_line)
            else:
                real_lines.append(prev_line)

    ##print("rl:",real_lines)
    return real_lines




def draw_lines(img, lines): 

    for line in lines:
        ##print(line)
        ##print("##########")
        x1, y1, x2, y2 = line[0]
        ##print("y:", y2-y1)
        ##print("x:", x2-x1)

        # 여러색으로 그림
        color = (random.randint(0,255), random.randint(0,255), random.randint(0,255))
        img = cv2.line(img, (x1, y1), (x2, y2), color, 2) 

    return img

# 평소에 사용하던 자표계와 달라서 right의 기울기가 양수, left의 기울기가 음수가 나옴.
def get_avg_gradient(lines):
    sum = 0
    for line in lines:
        x1, y1, x2, y2 = line[0]

        sum += float(y2-y1)/float(x2-x1)
    
    avg = sum / len(lines)

    return avg

def get_avg_line(lines):
    sum = [0, 0, 0, 0]
    for line in lines:
        x1, y1, x2, y2 = line[0]

        sum[0] += x1
        sum[1] += y1
        sum[2] += x2
        sum[3] += y2

    for i in range(len(sum)):
        sum[i] /= len(lines)

    return sum

# def get_avg_line2(lines): # 직선 앞의 세개만 평균냄. 별로
#     sum = [0, 0, 0, 0]
#     for line in lines[:3]:
#         x1, y1, x2, y2 = line[0]
#         sum[0] += x1
#         sum[1] += y1
#         sum[2] += x2
#         sum[3] += y2

#     for i in range(len(sum)):
#         sum[i] /= len(lines)

#     return sum

def show_gradient(img):
    left_text = 'left gradient:' + str(left_grad)
    right_text = 'right gradient:' + str(right_grad)

    cv2.putText(img, left_text, (10,200), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (153,0,255), 1)
    cv2.putText(img, right_text, (400,200), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (153,0,255), 1)

def update_grad_deque(grad):
    global grad_deque
    grad_deque.append(grad)
    total_sum = sum(grad_deque)
    avg_grad = float(total_sum) / float(len(grad_deque))

    return avg_grad


# You are to find "left and right position" of road lanes
def process_image(frame):
    global Offset, prev_lines_l, lines_l, prev_lines_r, lines_r, xl_1, xl_2, xl_3, xl_4, yl_1, yl_2, yl_3, yl_4, xr_1, xr_2, xr_3, xr_4, yr_1, yr_2, yr_3, yr_4, left_grad, right_grad
    src = frame.copy()
    #cv2.imshow("HoughLinesP", src)
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    cv2.imshow('gray',gray)
    #cv2.calibrateCamera()
    #cv2.undistort()

    # hsv는 효율 별로인듯
    # hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    # lower_white = (0, 0, 0)
    # upper_white = (360, 30, 80)
    # hsv_mask = cv2.inRange(hsv, lower_white, upper_white)
    # hsv_lane = cv2.bitwise_and(frame, frame, mask=hsv_mask)
    # cv2.imshow(hsv_lane)

    # ersoe, dilate
    kernel_row = 5
    kernel_col = 5
    kernel = np.ones((kernel_row, kernel_col), np.uint8)
    # kernel = [[0.,0.,0.,0.,1.,1.,0.,0.,0.,0.],
    #           [0.,0.,0.,1.,1.,1.,1.,0.,0.,0.],
    #           [0.,0.,1.,1.,1.,1.,1.,1.,0.,0.],
    #           [0.,1.,1.,1.,1.,1.,1.,1.,1.,0.],
    #           [1.,1.,1.,1.,1.,1.,1.,1.,1.,1.],
    #           [1.,1.,1.,1.,1.,1.,1.,1.,1.,1.],
    #           [0.,1.,1.,1.,1.,1.,1.,1.,1.,0.],
    #           [0.,0.,1.,1.,1.,1.,1.,1.,0.,0.],
    #           [0.,0.,0.,1.,1.,1.,1.,0.,0.,0.],
    #           [0.,0.,0.,0.,1.,1.,0.,0.,0.,0.]]
    #kernel = np.array(kernel)

    erosion_img = cv2.erode(gray, kernel, iterations=1)
    cv2.imshow('erose1', erosion_img)
    #erosion_img = cv2.erode(erosion_img, kernel, iterations=1)
    #cv2.imshow('erose2', erosion_img)
    #top_hat = gray - erosion_img
    #cv2.imshow('top-hat', top_hat)
    #bottom_hat = erosion_img - gray
    #cv2.imshow('bottom-hat', bottom_hat)
    dilation_img = cv2.dilate(erosion_img, kernel, iterations=1)
    cv2.imshow('dialate', dilation_img)
    lane_img = gray - dilation_img
    cv2.imshow('gray - diate', lane_img)


    # blur (cv2.bilateralFilter도 써보기)
    kernel_size = 5 # 3은 너무 블러처리 덜 되고(-> 중앙선 잘 잡음), 7은 너무 많이돼서 차선도 가끔 잃음
    blur = cv2.GaussianBlur(lane_img, (kernel_size, kernel_size), 0)
    cv2.imshow('blur', blur)

    # canny 
    low_threshold = 60 # 60까지 조절해가며 최적 찾기
    high_threshold = 80 # 얘도 너무 높으면 벽쪽 선도 인식함
    canny_left = cv2.Canny(np.uint8(blur), low_threshold, high_threshold)
    canny_right = canny_left.copy()

    # roi(꼭짓점 쓰는 순서도 중요함)
    vertices_left = np.array([[(xl_1, yl_1), (xl_2, yl_2), (xl_4, yl_4), (xl_3, yl_3)]], dtype=np.int32)
    left_roi = region_of_interest(canny_left, vertices_left, 255)
    vertices_right = np.array([[(xr_1, yr_1), (xr_2, yr_2), (xr_4, yr_4), (xr_3, yr_3)]], dtype=np.int32)
    right_roi = region_of_interest(canny_right, vertices_right, 255)
    cv2.imshow("left_roi", left_roi)
    cv2.imshow("right_roi", right_roi)

    # HoughLinesP
    lines_l = cv2.HoughLinesP(left_roi, 1, math.pi/180, 35, 2, 15) # 인자 조절

    lines_r = cv2.HoughLinesP(right_roi, 1, math.pi/180, 35, 2, 15)

    # process lines
    lines_l = process_lines(lines_l, 'left')
    lines_r = process_lines(lines_r, 'right')
    ##print("l:", lines_l)
    ##print("r:", lines_r)
    
    # print(lines)
    # print("!!")

    #line detect가 안 된 경우(S자 부분에 바닥이 되게 밝은 회색이라 흰색과 잘 구분되지 않는 부분이 존재)
    # if (not lines_l) and (lines_r):
	# ##print('1')
	#     lines_l = lines_r # 좋은 방법인듯. 검출을 못하면(곡선구간이라. 대부분 반대쪽 선은 있음.) 반대쪽의 이전 선(왜냐하면 반대쪽의 현재 선이 검출 안될 수 있음)로 대체
    # elif (not lines_r) and (lines_l):
	# ##print('2')	
	#     lines_r = lines_l
    # elif (not lines_l) and (not lines_r):
	# ##print('3')
    #     if (get_avg_gradient(prev_lines_l) >= 0 and get_avg_gradient(prev_lines_r) >= 0):	
	#         lines_l = [[[int(prev_lines_l[0][0][0]*0.9999999), int(prev_lines_l[0][0][1]*0.9999999), int(prev_lines_l[0][0][2]*0.9999999), int(prev_lines_l[0][0][3]*0.9999999)]]]
	#         lines_r = [[[int(prev_lines_r[0][0][0]*0.99), int(prev_lines_r[0][0][1]*0.9999999), int(prev_lines_r[0][0][2]*0.99), int(prev_lines_r[0][0][3]*0.9999999)]]]
    #             print("1")
    #     elif (get_avg_gradient(prev_lines_l) <= 0 and get_avg_gradient(prev_lines_r) <= 0):
	#         lines_l = [[[int(prev_lines_l[0][0][0]*1.0000001), int(prev_lines_l[0][0][1]*0.9999999), int(prev_lines_l[0][0][2]*1.0000001), int(prev_lines_l[0][0][3]*0.9999999)]]]
	#         lines_r = [[[int(prev_lines_r[0][0][0]*1.0000001), int(prev_lines_r[0][0][1]*0.9999999), int(prev_lines_r[0][0][2]*1.0000001), int(prev_lines_r[0][0][3]*0.9999999)]]]
    #             print("2")
    #     else:
    #         lines_l = prev_lines_l
    #         lines_r = prev_lines_r
    #         print("3")
    #     print('l:',lines_l)
    #     print('r:',lines_r)



    if (not lines_l) and (lines_r):
	lines_l = lines_r # 좋은 방법인듯. 검출을 못하면(곡선구간이라. 대부분 반대쪽 선은 있음.) 반대쪽의 이전 선(왜냐하면 반대쪽의 현재 선이 검출 안될 수 있음)로 대체
    elif (not lines_r) and (lines_l):
	lines_r = lines_l
    elif (not lines_l) and (not lines_r):
        lines_l = prev_lines_l
        lines_r = prev_lines_r

    # 아직도 중앙선 잡는 경우 처리
    print("ll",abs(lines_l[0][0][2] - prev_lines_l[0][0][2]))
    print("rr",abs(lines_r[0][0][2] - prev_lines_r[0][0][2]))
    if (abs(lines_l[0][0][2] - prev_lines_l[0][0][2])  > 35) and (abs(lines_l[0][0][2] - prev_lines_l[0][0][2]) < 380):
        lines_l = prev_lines_l
    if (abs(lines_r[0][0][2] - prev_lines_r[0][0][2])  > 35) and (abs(lines_r[0][0][2] - prev_lines_r[0][0][2]) < 380):
        lines_r = prev_lines_r


        

	#lines_l = prev_lines_r # 좋은 방법인듯. 검출을 못하면(곡선구간이라. 대부분 반대쪽 선은 있음.) 반대쪽의 이전 선(왜냐하면 반대쪽의 현재 선이 검출 안될 수 있음)로 대체
    #elif (not lines_r) and (not lines_l):
	#print('4')
	#lines_r = prev_lines_l

    prev_lines_l = lines_l
    prev_lines_r = lines_r


    #print("pl:", prev_lines_l)
    #print("pr:", prev_lines_r)
    #print("l:", lines_l)
    #print("r:", lines_r)

    # draw hough lines
    draw_lines(src, lines_l)
    draw_lines(src, lines_r)

    # find rectangle area_1 - use pixel threshold
    # lpos, rpos = 135, 550
    # roi_mid_row = 315
    # roi_mid_col = 335
    # start_l, start_r = 290, 380 # 조절
    # ##cv2.circle(src, (start_l, 315),5, (255, 0, 0), -1)
    # ##cv2.circle(src, (start_r, 315),5, (255, 0, 0), -1)
    # end_l, end_r = xl_2, xr_1
    # area_len = 10 # 픽셀 수 확인 영역 가로,세로 길이
    # pixel_cnt_threshold = area_len * area_len * 0.2
    # prev_lpos, prev_rpos = lpos, rpos # 직사각형 놓친 경우 대비

    # for l in range(start_l, end_l, -1):
    #     area = left_roi[roi_mid_row : roi_mid_row + area_len, l : l + area_len] # 행, 열
    #     if cv2.countNonZero(area) > pixel_cnt_threshold:
    #         lpos = l

    # for r in range(start_r, end_r):
    #     area = right_roi[roi_mid_row : roi_mid_row + area_len, r : r + area_len] # 행, 열
    #     if cv2.countNonZero(area) > pixel_cnt_threshold:
    #         rpos = r
    # ##print("l:", lpos)
    # ##print("r:", rpos)

    # if abs(prev_lpos - lpos) > 100: # 중간선 잡기 방지 
    #     lpos = prev_lpos
    # if abs(prev_rpos - rpos) > 100:
    #     rpos = prev_rpos

    #dst = draw_rectangle(dst, lpos, rpos, offset=Offset)


    # find rectangle area_2 - use lines
    left_line = []
    right_line = []


    # find average line
    left_line = get_avg_line(lines_l)
    right_line = get_avg_line(lines_r)

    # find average gradient
    left_grad = get_avg_gradient(lines_l)
    right_grad = get_avg_gradient(lines_r)

    # draw rectangle
    lpos = ((left_line[0]+left_line[2])/2, (left_line[1]+left_line[3])/2)
    rpos = ((right_line[0]+right_line[2])/2, (right_line[1]+right_line[3])/2)

    draw_rectangle(src, lpos[0], lpos[1])
    draw_rectangle(src, rpos[0], rpos[1])

    # draw blue line
    cv2.line(src, (left_line[0]+200,left_line[1]+int(left_grad*200)), (left_line[2],left_line[3]), (255,0,0), 3)
    cv2.line(src, (right_line[0]-200,right_line[1]-int(right_grad*200)), (right_line[2],right_line[3]), (255,0,0), 3)
    #cv2.imshow('src', src)



######### 대표 직선이랑 lpos, rpos 구하기 위해 가변적 roi를 써야할 것 같음, 지금은 그렇게 안 하지만 대회해서는 상황별로 roi 다르게 잡으면 좋을 것 같음. ##########




    # check ROI
    cv2.circle(src, (xl_1, yl_1),5, (0, 0, 255), -1)
    cv2.circle(src, (xl_2, yl_2),5, (0, 0, 255), -1)
    cv2.circle(src, (xl_3, yl_3),5, (0, 0, 255), -1)
    cv2.circle(src, (xl_4, yl_4),5, (0, 0, 255), -1)

    cv2.circle(src, (xr_1, yr_1),5, (0, 0, 255), -1)
    cv2.circle(src, (xr_2, yr_2),5, (0, 0, 255), -1)
    cv2.circle(src, (xr_3, yr_3),5, (0, 0, 255), -1)
    cv2.circle(src, (xr_4, yr_4),5, (0, 0, 255), -1)

    cv2.imshow("src", src)

    return (lpos, rpos), src

def get_steer_angle(img, pos):
    global left_grad, right_grad, steer_angle

    lpos = pos[0]
    rpos = pos[1]
    midpos = (lpos[0]+rpos[0])/2
    print(midpos)

    # 1. 기울기
    if (left_grad <= -0.65) and (right_grad >= 0.65): 
        #  2. 거리로 직진 보완
        if (midpos > 320):
            steer_angle = -5
            cv2.putText(img, "turn slightly right", (200,80), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255,255,255), 2)
        elif (midpos < 270):
            steer_angle = 5
            cv2.putText(img, "turn slightly left", (200,80), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255,255,255), 2)
        else:
            steer_angle = 0
            cv2.putText(img, "go straight", (250,80), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255,255,255), 2)

    elif left_grad > 0: # left를 잃음
        if right_grad >= 0.65:
            #  2. 거리로 직진 보완
            if (midpos > 320):
                steer_angle = -5
                cv2.putText(img, "turn slightly right", (200,80), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255,255,255), 2)
            elif (midpos < 270):
                steer_angle = 5
                cv2.putText(img, "turn slightly left", (200,80), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255,255,255), 2)
            else:
                steer_angle = 0
                cv2.putText(img, "go straight", (250,80), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255,255,255), 2)
        else:
            steer_angle = 4000/23*right_grad*right_grad - 8200/23*right_grad + 3560/23 - 10
            cv2.putText(img, "turn left", (250,80), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255,255,255), 2)
    elif right_grad < 0: # right를 잃음
        if left_grad <= -0.65:
            #  2. 거리로 직진 보완
            if (midpos > 320):
                steer_angle = -5
                cv2.putText(img, "turn slightly right", (200,80), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255,255,255), 2)
            elif (midpos < 270):
                steer_angle = 5
                cv2.putText(img, "turn slightly left", (200,80), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255,255,255), 2)
            else:
                steer_angle = 0
                cv2.putText(img, "go straight", (250,80), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255,255,255), 2)
        else:
            steer_angle =  -4000/23*left_grad*left_grad - 8200/23*left_grad - 3560/23
            cv2.putText(img, "turn right", (250,80), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255,255,255), 2)
    else:
        # 2. 거리로 직진 보완
        if (midpos > 320):
            steer_angle = -5
            cv2.putText(img, "turn slightly right", (200,80), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255,255,255), 2)
        elif (midpos < 270):
            steer_angle = 5
            cv2.putText(img, "turn slightly left", (200,80), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255,255,255), 2)
        else:
            steer_angle = 0
            cv2.putText(img, "go straight", (250,80), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255,255,255), 2)



    # if abs(left_grad) > 0.5 and abs(right_grad)> 0.5:
    #     steer_angle = -0.5 * (-125*right_grad*right_grad -90*right_grad + 106)
    # elif abs(left_grad) > abs(right_grad) - 10:
    #     steer_angle = -0.5 * (-125*right_grad*right_grad -90*right_grad + 106)
    # else:
    #     steer_angle = 0.5 * (-95*left_grad*left_grad -90*left_grad + 106)


    # if min(abs(left_grad), abs(right_grad)) < 0.5:
    #     steer_angle = min(abs(left_grad), abs(right_grad))
    return steer_angle


def draw_steer(image, steer_angle):
    global Width, Height, arrow_pic

    arrow_pic = cv2.imread('steer_arrow.png', cv2.IMREAD_COLOR)

    origin_Height = arrow_pic.shape[0]
    origin_Width = arrow_pic.shape[1]
    steer_wheel_center = origin_Height * 0.74
    arrow_Height = Height/2
    arrow_Width = (arrow_Height * 462)/728

    matrix = cv2.getRotationMatrix2D((origin_Width/2, steer_wheel_center), (steer_angle) * 1.5, 0.7)
    arrow_pic = cv2.warpAffine(arrow_pic, matrix, (origin_Width+60, origin_Height)) # 어파인 변환한 >결과 영상을 생성
    arrow_pic = cv2.resize(arrow_pic, dsize=(arrow_Width, arrow_Height), interpolation=cv2.INTER_AREA)

    gray_arrow = cv2.cvtColor(arrow_pic, cv2.COLOR_BGR2GRAY)
    _, mask = cv2.threshold(gray_arrow, 1, 255, cv2.THRESH_BINARY_INV)

    arrow_roi = image[arrow_Height: Height, (Width/2 - arrow_Width/2) : (Width/2 + arrow_Width/2)]
    #cv2.imshow('1', arrow_roi)
    arrow_roi = cv2.add(arrow_pic, arrow_roi, mask=mask)
    #cv2.imshow('2', arrow_roi)
    res = cv2.add(arrow_roi, arrow_pic)
    #cv2.imshow('3', res)
    image[(Height - arrow_Height): Height, (Width/2 - arrow_Width/2): (Width/2 + arrow_Width/2)] = res

    cv2.imshow('steer', image)


    
# You are to publish "steer_anlge" following load lanes
if __name__ == '__main__':
    cap = cv2.VideoCapture('kmu_track.mkv')
    time.sleep(3)

    while not rospy.is_shutdown():
        time1 = time.time()
        ret, image = cap.read()
        pos, frame = process_image(image) # pos = (lpos, rpos), frame은 가공된 frame
        # show gradient
        show_gradient(frame)
        steer_angle = get_steer_angle(frame, pos)
        draw_steer(frame, steer_angle)
        time2 = time.time()
        print("time:",time1-time2)

        if cv2.waitKey(3) & 0xFF == ord('q'):
            break

