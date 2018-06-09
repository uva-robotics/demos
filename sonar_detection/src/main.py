#!/usr/bin/python

import rospy
from std_msgs.msg import String
import time
from sensor_msgs.msg import Range

# def sonar_front(data):
#     print(data.range)

def sonar_back(data):
    print(data.range)


if __name__ == '__main__':
    rospy.init_node("Chris_main")
    publisher = rospy.Publisher("/speech", String, queue_size=10)
    # publisher.publish("Hello, i'm Pepper")

    rospy.Subscriber("/pepper_robot/sonar/back", Range, sonar_back)
    # rospy.Subscriber("/pepper_robot/sonar/front", Range, sonar_front)
    rospy.spin()
