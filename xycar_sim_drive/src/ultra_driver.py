#!/usr/bin/env python
# xycar_msg = (fl, fm, fr, 0, 0, 0, r, l)
import rospy, math
import time
from std_msgs.msg import Int32MultiArray

# ultra_sub = None
motor_pub = None
usonic_data = None


def init_node():
    global motor_pub
    rospy.init_node('guide')
    rospy.Subscriber('ultrasonic', Int32MultiArray, callback)
    motor_pub = rospy.Publisher('xycar_motor_msg', Int32MultiArray, queue_size=1)


def callback(data):
    global usonic_data
    usonic_data = data.data
    # print("Here!! usonic", usonic_data)
    # print(msg.data)
    return usonic_data


def drive(angle, speed):
    global motor_pub
    drive_info = [angle, speed]
    pub_data = Int32MultiArray(data=drive_info)
    motor_pub.publish(pub_data)


# xycar_msg = Int32MultiArray()
# u_data = msg.data

if __name__ == '__main__':
    init_node()
    time.sleep(1)
    rate = rospy.Rate(10)
    angle = 0
    while not rospy.is_shutdown():
        global usonic_data
        print("Here!! usonic", usonic_data)

        if usonic_data[0] < usonic_data[2]:
            angle = (usonic_data[2] - usonic_data[0])
            drive(angle, 60)
        elif usonic_data[2] < usonic_data[0]:
            angle = (usonic_data[2] - usonic_data[0])
            drive(angle, 60)
        else:
            drive(0, 120)


