#! /usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import actionlib
from PBVS import PBVS
import forklift_server.msg
import tf
from apriltag_ros.msg import AprilTagDetectionArray
from nav_msgs.msg import Odometry
import math
from gpm_msg.msg import forkposition
from ekf import KalmanFilter
import tkinter as tk

class Subscriber():
    def __init__(self):
        odom = rospy.get_param(rospy.get_name() + "/odom", "/odom")
        tag_detections_up = rospy.get_param(rospy.get_name() + "/tag_detections_up", "/tag_detections_up")
        tag_detections_down = rospy.get_param(rospy.get_name() + "/tag_detections_down", "/tag_detections_down")
        forkpos = rospy.get_param(rospy.get_name() + "/forkpos", "/forkpos")
        self.sub_info_marker = rospy.Subscriber(tag_detections_up, AprilTagDetectionArray, self.cbGetMarker_up, queue_size = 1)
        self.sub_info_marker = rospy.Subscriber(tag_detections_down, AprilTagDetectionArray, self.cbGetMarker_down, queue_size = 1)
        self.sub_odom_robot = rospy.Subscriber(odom, Odometry, self.cbGetRobotOdom, queue_size = 1)
        self.sub_forwardbackpostion = rospy.Subscriber(forkpos, forkposition, self.cbGetforkpos, queue_size = 1)
        self.ekf_theta = KalmanFilter()
        self.init_parame()

        if(rospy.get_param(rospy.get_name() + "/gui", False)):
            self.windows()

    def init_parame(self):
        # Odometry_param
        self.is_odom_received = False
        self.robot_2d_pose_x = 0.0
        self.robot_2d_pose_y = 0.0
        self.robot_2d_theta = 0.0
        self.previous_robot_2d_theta = 0.0
        self.total_robot_2d_theta = 0.0
        # AprilTag_param
        self.updown = False
        self.marker_2d_pose_x = 0.0
        self.marker_2d_pose_y = 0.0
        self.marker_2d_theta = 0.0
        # Forklift_param
        self.forwardbackpostion = 0.0
        self.updownposition = 0.0
        #ekf
        self.ekf_theta.init(1,1,5)
    def __del__(self):
        self.window.destroy()

    def SpinOnce(self):
        return self.robot_2d_pose_x, self.robot_2d_pose_y, self.robot_2d_theta, \
               self.marker_2d_pose_x, self.marker_2d_pose_y, self.marker_2d_theta
    def SpinOnce_fork(self):
        return self.forwardbackpostion, self.updownposition

    def cbGetMarker_up(self, msg):
        try:
            if self.updown == True:
                # print("up tag")
                marker_msg = msg.detections[0].pose.pose.pose
                quaternion = (marker_msg.orientation.x, marker_msg.orientation.y, marker_msg.orientation.z, marker_msg.orientation.w)
                theta = tf.transformations.euler_from_quaternion(quaternion)[1]
                theta = self.ekf_theta.update(theta)
                self.marker_2d_pose_x = -marker_msg.position.z
                self.marker_2d_pose_y = marker_msg.position.x
                self.marker_2d_theta = -theta
            else:
                pass
        except:
            pass

    def cbGetMarker_down(self, msg):
        try:
            if self.updown == False:
                # print("down tag")
                marker_msg = msg.detections[0].pose.pose.pose
                quaternion = (marker_msg.orientation.x, marker_msg.orientation.y, marker_msg.orientation.z, marker_msg.orientation.w)
                theta = tf.transformations.euler_from_quaternion(quaternion)[1]
                theta = self.ekf_theta.update(theta)
                self.marker_2d_pose_x = -marker_msg.position.z
                self.marker_2d_pose_y = marker_msg.position.x
                self.marker_2d_theta = -theta

            else:
                pass
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

    def windows(self):

        self.window = tk.Tk()
        self.window.geometry('220x170+1700+560') 

        self.labelrobot_2d_pose_x = tk.Label(self.window, text="", font=('Helvetica', 10), fg='black')
        self.label_robot_2d_pose_x = tk.Label(self.window, text="", font=('Helvetica', 10), fg='black') 

        self.labelrobot_2d_pose_y= tk.Label(self.window, text="", font=('Helvetica', 10), fg='black')
        self.label_robot_2d_pose_y = tk.Label(self.window, text="", font=('Helvetica', 10), fg='black')  

        self.labelrobot_2d_theta= tk.Label(self.window, text="", font=('Helvetica', 10), fg='black')
        self.label_robot_2d_theta = tk.Label(self.window, text="", font=('Helvetica', 10), fg='black')   

        self.labelmarker_2d_pose_x= tk.Label(self.window, text="", font=('Helvetica', 10), fg='black')
        self.label_marker_2d_pose_x = tk.Label(self.window, text="", font=('Helvetica', 10), fg='black')        

        self.labelmarker_2d_pose_y= tk.Label(self.window, text="", font=('Helvetica', 10), fg='black')
        self.label_marker_2d_pose_y = tk.Label(self.window, text="", font=('Helvetica', 10), fg='black')     

        self.labelmarker_2d_theta= tk.Label(self.window, text="", font=('Helvetica', 10), fg='black')
        self.label_marker_2d_theta = tk.Label(self.window, text="", font=('Helvetica', 10), fg='black') 

        self.labelfork_pose_updownposition= tk.Label(self.window, text="", font=('Helvetica', 10), fg='black')
        self.label_fork_pose_updownposition = tk.Label(self.window, text="", font=('Helvetica', 10), fg='black') 
        
        self.labelfork_pose_forwardbackpostion= tk.Label(self.window, text="", font=('Helvetica', 10), fg='black')
        self.label_fork_pose_forwardbackpostion = tk.Label(self.window, text="", font=('Helvetica', 10), fg='black') 
        self.update_window()
        self.window.mainloop()

    def update_window(self):
        base = 0
        
        base1 = base
        self.labelrobot_2d_pose_x.configure(text='Robot 2d Pose x: ')
        self.labelrobot_2d_pose_x.place(x=0, y=base1)        
        self.label_robot_2d_pose_x.configure(text=self.robot_2d_pose_x)
        self.label_robot_2d_pose_x.place(x=190, y=base1)

        self.labelrobot_2d_pose_y.configure(text="Robot 2d Pose y: ")
        self.labelrobot_2d_pose_y.place(x=0, y=base1+20)
        self.label_robot_2d_pose_y.place(x=190, y=base1+20)
        self.label_robot_2d_pose_y.configure(text=self.robot_2d_pose_y)

        self.labelrobot_2d_theta.configure(text="Robot 2d theta: ")
        self.labelrobot_2d_theta.place(x=0, y=base1+40)
        self.label_robot_2d_theta.place(x=190, y=base1+40)
        self.label_robot_2d_theta.configure(text=math.degrees(self.robot_2d_theta))

        base2 = base1+60
        self.labelmarker_2d_pose_x.configure(text="Marker 2d Pose x: ")
        self.labelmarker_2d_pose_x.place(x=0, y=base2)
        self.label_marker_2d_pose_x.place(x=190, y=base2)
        self.label_marker_2d_pose_x.configure(text=self.marker_2d_pose_x)

        self.labelmarker_2d_pose_y.configure(text="Marker 2d Pose y: ")
        self.labelmarker_2d_pose_y.place(x=0, y=base2+20)
        self.label_marker_2d_pose_y.place(x=190, y=base2+20)
        self.label_marker_2d_pose_y.configure(text=self.marker_2d_pose_y)

        self.labelmarker_2d_theta.configure(text="Marker 2d theta: ")
        self.labelmarker_2d_theta.place(x=0, y=base2+40)
        self.label_marker_2d_theta.place(x=190, y=base2+40)
        self.label_marker_2d_theta.configure(text=self.marker_2d_theta)

        self.labelfork_pose_updownposition.configure(text="Fork updown position: ")
        self.labelfork_pose_updownposition.place(x=0, y=base2+60)
        self.label_fork_pose_updownposition.place(x=190, y=base2+60)
        self.label_fork_pose_updownposition.configure(text=self.updownposition)

        self.labelfork_pose_forwardbackpostion.configure(text="Fork forwardback position: ")
        self.labelfork_pose_forwardbackpostion.place(x=0, y=base2+80)
        self.label_fork_pose_forwardbackpostion.place(x=190, y=base2+80)
        self.label_fork_pose_forwardbackpostion.configure(text=self.forwardbackpostion)

        self.window.after(50, self.update_window)

class PBVSAction():
    def __init__(self, name):
        self.subscriber = Subscriber()
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, forklift_server.msg.PBVSAction, execute_cb=self.execute_cb, auto_start = False)
        self._result = forklift_server.msg.PBVSResult()
        self._as.start()

    def execute_cb(self, msg):
        rospy.loginfo('PBVS receive command : %s' % (msg))
        
        self.PBVS = PBVS(self._as, self.subscriber, msg)
        rospy.logwarn('PBVS Succeeded')
        self._result.result = 'PBVS Succeeded'
        self.subscriber.updown = True
        self._as.set_succeeded(self._result)
        self.PBVS = None


if __name__ == '__main__':
    rospy.init_node('PBVS_server')
    rospy.logwarn(rospy.get_name() + 'start')
    server = PBVSAction(rospy.get_name())
    rospy.spin()
