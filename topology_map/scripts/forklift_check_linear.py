#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
class CalibrateLinear():
    def __init__(self):
        # Give the node a name
        rospy.init_node('forklift_check_linear', anonymous=False)
        sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
        self.ori_x = 0
        self.odom_x = 0
        r = rospy.Rate(100)
        self.trigger = False
        while True:
            if abs(self.odom_x-self.ori_x) > 1.0:
                cmd_vel.publish(Twist())
                print("break")
                break
            print((self.ori_x - self.odom_x))
            move_cmd = Twist()
            move_cmd.linear.x = 0.15
            cmd_vel.publish(move_cmd)
            r.sleep()
        
    def odom_callback(self, msg):
        if not self.trigger:
            self.trigger = True
            self.ori_x = msg.pose.pose.position.x
        self.odom_x = msg.pose.pose.position.x

    def spin(self):
        rospy.spin()

if __name__ == '__main__':
    node = CalibrateLinear()
    node.spin()

