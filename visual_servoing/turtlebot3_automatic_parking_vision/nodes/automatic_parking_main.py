#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from apriltag_ros.msg import AprilTagDetectionArray
from automatic_parking_control import param, windows
import tf
import rospy
import numpy as np



class subscriber(param):
    def __init__(self):
        self.sub_info_marker = rospy.Subscriber('/tag_detections', AprilTagDetectionArray, self.cbGetMarker_up, queue_size = 1)
        
    def cbGetMarker_up(self, msg):
        try:
            marker_msg = msg.detections[0].pose.pose.pose
            quaternion = (marker_msg.orientation.x, marker_msg.orientation.y, marker_msg.orientation.z, marker_msg.orientation.w)
        except:
            rospy.sleep(0.5)
        theta = tf.transformations.euler_from_quaternion(quaternion)[1]
        #marker_2d_pose_x AprilTag距離相機多遠(cm)
        param.marker_2d_pose_x = marker_msg.position.z
        #marker_2d_pose_y AprilTag距離相機的左右距離 (左為負, 右為正)(cm)
        param.marker_2d_pose_y = marker_msg.position.x
        #marker_2d_pose_y AprilTag與相機夾角(由上往下看, 順時針為正, 逆時針為負)(degree)
        param.marker_2d_theta  = theta*180/np.pi
        # print (param.marker_2d_pose_x)
        # print (param.marker_2d_pose_y)
        # print (param.marker_2d_theta)

    def spin(self):
        rospy.spin()


if __name__ == '__main__':
    rospy.init_node('pbvs')
    subscriber = subscriber()
    windows = windows()
    subscriber.spin()
