#! /usr/bin/env python

import rospy, math
import cv2, time, rospy
import numpy as np

from ar_track_alvar_msgs.msg import AlvarMarkers

from tf.transformations import euler_from_quaternion

from std_msgs.msg import Int32MultiArray

arData = {"DX":0.0, "DY":0.0, "DZ":0.0, "AX":0.0, "AY":0.0, "AZ":0.0, "AW":0.0}

roll, pitch, yaw = 0, 0, 0

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

rospy.init_node('ar_drive')

rospy.Subscriber('ar_pose_marker', AlvarMarkers, callback)

motor_pub = rospy.Publisher('xycar_motor_msg', Int32MultiArray, queue_size =1 )

xycar_msg = Int32MultiArray()


while not rospy.is_shutdown():

    (roll,pitch,yaw)=euler_from_quaternion((arData["AX"],arData["AY"],arData["AZ"], arData["AW"]))
	
    roll = math.degrees(roll)
    pitch = math.degrees(pitch)
    yaw = math.degrees(yaw)
    img = np.zeros((100, 500, 3))

    img = cv2.line(img,(25,65),(475,65),(0,0,255),2)
    img = cv2.line(img,(25,40),(25,90),(0,0,255),3)
    img = cv2.line(img,(250,40),(250,90),(0,0,255),3)
    img = cv2.line(img,(475,40),(475,90),(0,0,255),3)

    point = int(arData["DX"]) + 250

    if point > 475:
        point = 475

    elif point < 25 : 
        point = 25	

    img = cv2.circle(img,(point,65),15,(0,255,0),-1)  
  
    distance = math.sqrt(pow(arData["DX"],2) + pow(arData["DY"],2))
    
    cv2.putText(img, str(int(distance))+" pixel", (350,25), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255))

    dx_dy_yaw = "DX:"+str(int(arData["DX"]))+" DY:"+str(int(arData["DY"])) \
                +" Yaw:"+ str(round(yaw,1)) 
    cv2.putText(img, dx_dy_yaw, (20,25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255))

    cv2.imshow('AR Tag Position', img)
    cv2.waitKey(1)

    angle = 50
    speed = 5

    xycar_msg.data = [angle, speed]
    motor_pub.publish(xycar_msg)

cv2.destroyAllWindows()




