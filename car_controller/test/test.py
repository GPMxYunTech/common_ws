#! /usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import math
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry


class TestNode(object):
    def __init__(self):
        rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.callback)
        self.last_x = 0
        self.last_y = 0
        self.last_time = rospy.Time.now()

    def callback(self, msg):
        print(self.last_x, ",", self.last_y)
        # print("x ", msg.pose.pose.position.x, self.last_x)
        # print("y ", msg.pose.pose.position.y, self.last_y)
        # print("time ", (rospy.Time.now() - self.last_time).to_sec())
        v = math.sqrt((msg.pose.pose.position.x - self.last_x)**2 + (msg.pose.pose.position.y - self.last_y)**2) / (rospy.Time.now() - self.last_time).to_sec()
        self.last_x = msg.pose.pose.position.x
        self.last_y = msg.pose.pose.position.y
        self.last_time = rospy.Time.now()
        # print("v ", v)

if __name__ == '__main__':
    rospy.init_node('test_node')
    node = TestNode()
    rospy.spin()


    # 驗證angular.z的值是否為設定值
    # pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    # rospy.sleep(math.pi*2 / 0.2)
    # twist = Twist()
    # pub.publish(twist)

