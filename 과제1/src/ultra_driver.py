import rospy
import time
from std_msgs.msg import Int32MultiArray
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


def drive(angle, speed):
    global motor_pub
    drive_info = [angle, speed]
    pub_data = Int32MultiArray(data=drive_info)
    motor_pub.publish(pub_data)


if __name__ == '__main__':
    init_node()
    time.sleep(1)

    while not rospy.is_shutdown():
        global usonic_data
        # turn right
        if usonic_data[0] < usonic_data[2]:
            angle = usonic_data[2] - usonic_data[0]
            drive(angle, 100)
        # turn left
        elif usonic_data[2] < usonic_data[0]:
            angle = usonic_data[2] - usonic_data[0]
            drive(angle, 100)
        # go straight
        else:
            drive(0, 100)
