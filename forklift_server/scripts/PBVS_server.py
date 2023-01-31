#! /usr/bin/env python3

import rospy
import actionlib
from PBVS import PBVS
import forklift_server.msg
import tf
from apriltag_ros.msg import AprilTagDetectionArray
from nav_msgs.msg import Odometry
import math
from gpm_msg.msg import forkposition

class Subscriber():
    def __init__(self):
        self.sub_info_marker = rospy.Subscriber('/tag_detections', AprilTagDetectionArray, self.cbGetMarker, queue_size = 1)
        self.sub_odom_robot = rospy.Subscriber('/wheel_odom', Odometry, self.cbGetRobotOdom, queue_size = 1)
        self.sub_forwardbackpostion = rospy.Subscriber('/forkpos', forkposition, self.cbGetforkpos, queue_size = 1)
        self.init_parame()

    def init_parame(self):
        # Odometry_param
        self.is_odom_received = False
        self.robot_2d_pose_x = 0.0
        self.robot_2d_pose_y = 0.0
        self.robot_2d_theta = 0.0
        self.previous_robot_2d_theta = 0.0
        self.total_robot_2d_theta = 0.0
        # AprilTag_param
        self.is_marker_pose_received = False
        self.marker_2d_pose_x = 0.0
        self.marker_2d_pose_y = 0.0
        self.marker_2d_theta = 0.0
        # Forklift_param
        self.forwardbackpostion = 0.0
        self.updownposition = 0.0

    def SpinOnce(self):
        return self.robot_2d_pose_x, self.robot_2d_pose_y, self.robot_2d_theta, \
               self.marker_2d_pose_x, self.marker_2d_pose_y, self.marker_2d_theta
    def SpinOnce_fork(self):
        return self.forwardbackpostion, self.updownposition

    def cbGetMarker(self, msg):
        try:
            if self.is_marker_pose_received == False:
                self.is_marker_pose_received = True
            marker_msg = msg.detections[0].pose.pose.pose
            quaternion = (marker_msg.orientation.x, marker_msg.orientation.y, marker_msg.orientation.z, marker_msg.orientation.w)
            theta = tf.transformations.euler_from_quaternion(quaternion)[1]
            self.marker_2d_pose_x = -marker_msg.position.z
            self.marker_2d_pose_y = marker_msg.position.x
            self.marker_2d_theta = -theta
        except:
            pass

    def cbGetRobotOdom(self, msg):
        if self.is_odom_received == False:
            self.is_odom_received = True 

        quaternion = (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
        theta = tf.transformations.euler_from_quaternion(quaternion)[2]
        if theta < 0:
            theta = theta + math.pi * 2
        if theta > math.pi * 2:
            theta = theta - math.pi * 2

        self.robot_2d_pose_x = msg.pose.pose.position.x
        self.robot_2d_pose_y = msg.pose.pose.position.y
        self.robot_2d_theta = theta

        if (self.robot_2d_theta - self.previous_robot_2d_theta) > 5.:
            d_theta = (self.robot_2d_theta - self.previous_robot_2d_theta) - 2 * math.pi
        elif (self.robot_2d_theta - self.previous_robot_2d_theta) < -5.:
            d_theta = (self.robot_2d_theta - self.previous_robot_2d_theta) + 2 * math.pi
        else:
            d_theta = (self.robot_2d_theta - self.previous_robot_2d_theta)

        self.total_robot_2d_theta = self.total_robot_2d_theta + d_theta
        self.previous_robot_2d_theta = self.robot_2d_theta

        self.robot_2d_theta = self.total_robot_2d_theta

    def cbGetforkpos(self, msg):
        self.forwardbackpostion = msg.forwardbackpostion
        self.updownposition = msg.updownposition

class PBVSAction():
    def __init__(self, name):
        self.subscriber = Subscriber()
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, forklift_server.msg.PBVSAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()
      
    def execute_cb(self, msg):
        rospy.loginfo('PBVS receive command : %s' % (msg))
        #TODO 對位與取放貨分開
        #TODO 棧板高度參數要加 
        
        # if self._as.is_preempt_requested():
        #     del self.PBVS
        #     rospy.loginfo('%s: Preempted' % self._action_name)
        #     self._as.set_preempted()
        #     success = False
        self.PBVS = PBVS(self._as, self.subscriber, int(msg.command))
        #up 8
        #down 14
        self.PBVS = None

if __name__ == '__main__':
    rospy.init_node('PBVS')
    server = PBVSAction(rospy.get_name())
    rospy.spin()
