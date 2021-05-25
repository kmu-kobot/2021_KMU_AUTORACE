#! /usr/bin/env python
import rospy
import math
import cv2
import time
import rospy
import numpy as np
from ar_track_alvar_msgs.msg import AlvarMarkers
from tf.transformations import euler_from_quaternion
from std_msgs.msg import Int32MultiArray
arData = {"DX": 0.0, "DY": 0.0, "DZ": 0.0,
          "AX": 0.0, "AY": 0.0, "AZ": 0.0, "AW": 0.0}
roll, pitch, yaw = 0, 0, 0
motor_pub = None
xycar_msg = None
speed = 0

def init_node():
    global motor_pub, xycar_msg
    rospy.init_node('ar_drive')
    rospy.Subscriber('ar_pose_marker', AlvarMarkers, callback)
    motor_pub = rospy.Publisher(
        'xycar_motor_msg', Int32MultiArray, queue_size=1)
    xycar_msg = Int32MultiArray()

def callback(msg):
    global arData
    for i in msg.markers:
        arData["DX"] = i.pose.pose.position.x
        arData["DY"] = i.pose.pose.position.y
        arData["DZ"] = i.pose.pose.position.z
        arData["AX"] = i.pose.pose.orientation.x
        arData["AY"] = i.pose.pose.orientation.y
        arData["AZ"] = i.pose.pose.orientation.z
        arData["AW"] = i.pose.pose.orientation.w

def artag_show(yaw):
    (roll, pitch, yaw) = euler_from_quaternion(
            (arData["AX"], arData["AY"], arData["AZ"], arData["AW"]))
    roll = math.degrees(roll)
    pitch = math.degrees(pitch)
    yaw = math.degrees(yaw)
    img = np.zeros((100, 500, 3))
    img = cv2.line(img, (25, 65), (475, 65), (0, 0, 255), 2)
    img = cv2.line(img, (25, 40), (25, 90), (0, 0, 255), 3)
    img = cv2.line(img, (250, 40), (250, 90), (0, 0, 255), 3)
    img = cv2.line(img, (475, 40), (475, 90), (0, 0, 255), 3)
    point = int(arData["DX"]) + 250
    if point > 475:
        point = 475
    elif point < 25:
        point = 25
    img = cv2.circle(img, (point, 65), 15, (0, 255, 0), -1)
    distance = 100
    distance = math.sqrt(pow(arData["DX"], 2) + pow(arData["DY"], 2))
    cv2.putText(img, str(int(distance))+" pixel", (350, 25),
                cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255))
    dx_dy_yaw = "DX:"+str(int(arData["DX"]))+" DY:"+str(int(arData["DY"])) \
                + " Yaw:" + str(round(yaw, 1))
    cv2.putText(img, dx_dy_yaw, (20, 25),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255))
    cv2.imshow('AR Tag Position', img)
    cv2.waitKey(1)

def back_drive(angle, cnt, yaw):
    global xycar_msg, motor_pub, speed
    speed = -10
    for i in range(cnt):
        xycar_msg.data = [angle, speed]
        motor_pub.publish(xycar_msg)
        init_node()
        artag_show(yaw)
        time.sleep(0.1)

    for j in range(cnt / 2):
        xycar_msg.data = [-angle, speed]
        motor_pub.publish(xycar_msg)
        init_node()
        artag_show(yaw)
        time.sleep(0.1)

if __name__ == '__main__':
    init_node()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        (roll, pitch, yaw) = euler_from_quaternion(
            (arData["AX"], arData["AY"], arData["AZ"], arData["AW"]))
        roll = math.degrees(roll)
        pitch = math.degrees(pitch)
        yaw = math.degrees(yaw)
        img = np.zeros((100, 500, 3))
        img = cv2.line(img, (25, 65), (475, 65), (0, 0, 255), 2)
        img = cv2.line(img, (25, 40), (25, 90), (0, 0, 255), 3)
        img = cv2.line(img, (250, 40), (250, 90), (0, 0, 255), 3)
        img = cv2.line(img, (475, 40), (475, 90), (0, 0, 255), 3)
        point = int(arData["DX"]) + 250
        if point > 475:
            point = 475
        elif point < 25:
            point = 25
        img = cv2.circle(img, (point, 65), 15, (0, 255, 0), -1)
        distance = 100
        distance = math.sqrt(pow(arData["DX"], 2) + pow(arData["DY"], 2))
        cv2.putText(img, str(int(distance))+" pixel", (350, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255))
        dx_dy_yaw = "DX:"+str(int(arData["DX"]))+" DY:"+str(int(arData["DY"])) \
                    + " Yaw:" + str(round(yaw, 1))
        cv2.putText(img, dx_dy_yaw, (20, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255))
        cv2.imshow('AR Tag Position', img)
        cv2.waitKey(1)
        angle = 50
        speed = 40
        yaw = math.radians(yaw)
        dx = arData["DX"]
        dy = arData["DY"]
        if yaw == 0:
            if dx > 0:
                angle = math.atan2(dx, dy)
            elif dx < 0:
                angle = math.atan2(dx, dy)
            elif dx == 0:
                angle = 0
        else:
            if dx == 0:
                angle = -yaw
            else:
                angle = math.atan2(dx, dy) - yaw

        if distance <= 72:
            speed = 40
            if (200 > dx and dx > 10) or (20 > math.degrees(yaw) and math.math.degrees(yaw) > 1):
                back_drive(50, 20, yaw)
            elif (-200 < dx and dx < -10) or (-20 < math.degrees(yaw) and math.math.degrees(yaw) < -1):
                back_drive(-50, 20, yaw)
            else:
                speed = 0

    angle = math.degrees(angle * 2.5)
    print('yaw->{}, dx->{}, distance->{}, angle->{}'.format(math.degrees(yaw), dx, distance, angle))
    xycar_msg.data = [angle, speed]
    motor_pub.publish(xycar_msg)
    
cv2.destroyAllWindows()