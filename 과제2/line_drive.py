#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import cv2, random, math, time

Width = 640
Height = 480
Offset = 330

# draw rectangle
def draw_rectangle(img, lpos, rpos, offset=0):
    center = (lpos + rpos) / 2

    cv2.rectangle(img, (lpos - 5, 15 + offset),
                       (lpos + 5, 25 + offset),
                       (0, 255, 0), 2)
    cv2.rectangle(img, (rpos - 5, 15 + offset),
                       (rpos + 5, 25 + offset),
                       (0, 255, 0), 2)   
    return img

# You are to find "left and light position" of road lanes
def process_image(frame):
    global Offset
    
    lpos, rpos = 100, 500
    frame = draw_rectangle(frame, lpos, rpos, offset=Offset)
    
    return (lpos, rpos), frame


def draw_steer(image, steer_angle):
    global Width, Height, arrow_pic

    arrow_pic = cv2.imread('steer_arrow.png', cv2.IMREAD_COLOR)

    origin_Height = arrow_pic.shape[0]
    origin_Width = arrow_pic.shape[1]
    steer_wheel_center = origin_Height * 0.74
    arrow_Height = Height/2
    arrow_Width = (arrow_Height * 462)/728

    matrix = cv2.getRotationMatrix2D((origin_Width/2, steer_wheel_center), (steer_angle) * 1.5, 0.7)    
    arrow_pic = cv2.warpAffine(arrow_pic, matrix, (origin_Width+60, origin_Height))
    arrow_pic = cv2.resize(arrow_pic, dsize=(arrow_Width, arrow_Height), interpolation=cv2.INTER_AREA)

    gray_arrow = cv2.cvtColor(arrow_pic, cv2.COLOR_BGR2GRAY)
    _, mask = cv2.threshold(gray_arrow, 1, 255, cv2.THRESH_BINARY_INV)

    arrow_roi = image[arrow_Height: Height, (Width/2 - arrow_Width/2) : (Width/2 + arrow_Width/2)]
    arrow_roi = cv2.add(arrow_pic, arrow_roi, mask=mask)
    res = cv2.add(arrow_roi, arrow_pic)
    image[(Height - arrow_Height): Height, (Width/2 - arrow_Width/2): (Width/2 + arrow_Width/2)] = res

    cv2.imshow('steer', image)

# You are to publish "steer_anlge" following load lanes
if __name__ == '__main__':
    cap = cv2.VideoCapture('kmu_track.mkv')
    time.sleep(3)

    while not rospy.is_shutdown():
        ret, image = cap.read()
        pos, frame = process_image(image)
        
        steer_angle = 0
        draw_steer(frame, steer_angle)

        if cv2.waitKey(3) & 0xFF == ord('q'):
            break

