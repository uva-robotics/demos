#!/usr/bin/python

import rospy
import time
from std_msgs.msg import String
from sensor_msgs.msg import Range
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

def sonar_front(data):
    print(data.range)
    if data.range < 0.0:
        publisher_velocity.publish("linear: \
        x: 0.0 \
        y: 0.0 \
        z: 0.0 \
        angular: \
        x:0.0 \
        y:0.0 \
        z:0.0")
        publisher_speech.publish("Pardon me, could you please move aside?")
    time.sleep(3)

# def sonar_back(data):
#     print(data.range)

# def laser_detector(data):
#     print(data.ranges)
#     time.sleep(1)


if __name__ == '__main__':
    rospy.init_node("Chris_main")
    publisher_speech = rospy.Publisher("/speech", String, queue_size=10)
    publisher_velocity = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
    # rospy.Subscriber("/pepper_robot/sonar/back", Range, sonar_back)
    rospy.Subscriber("/pepper_robot/sonar/front", Range, sonar_front)
    # rospy.Subscriber("/pepper_robot/laser", LaserScan, laser_detector)
    rospy.spin()
