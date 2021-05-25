#!/usr/bin/env python

import rospy, math
from std_msgs.msg import Int32MultiArray

def callback(msg):
    print(msg.data)

rospy.init_node('guide')
motor_pub = rospy.Publisher('xycar_motor_msg', Int32MultiArray, queue_size=1)
ultra_sub = rospy.Subscriber('ultrasonic', Int32MultiArray, callback)

xycar_msg = Int32MultiArray()

while not rospy.is_shutdown():
    angle = 0
    xycar_msg.data = [angle, 10]
    motor_pub.publish(xycar_msg)
