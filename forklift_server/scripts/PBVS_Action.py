# -*- coding: utf-8 -*-
import rospy
import numpy as np
import math
from geometry_msgs.msg import Twist
from enum import Enum
from gpm_msg.msg import forklift
class Action():
    def __init__(self, Subscriber):
        # cmd_vel
        self.cmd_vel = cmd_vel()
        self.Subscriber = Subscriber
        self.NearbySequence = Enum('NearbySequence', 'initial_turn go_straight turn_right parking ')
        self.current_nearby_sequence = self.NearbySequence.initial_turn.value
        # fork_cmd
        self.pub_fork = rospy.Publisher('/cmd_fork', forklift, queue_size = 1)
        self.forkmotion = Enum('forkmotion', 
                        'stop \
                        up \
                        down \
                        forward \
                        backword \
                        tilt_forward \
                        tilt_backword')
        # Odometry_param
        self.is_odom_received = False
        self.robot_2d_pose_x = 0.0
        self.robot_2d_pose_y = 0.0
        self.robot_2d_theta = 0.0
        # AprilTag_param
        self.is_marker_pose_received = False
        self.marker_2d_pose_x = 0.0
        self.marker_2d_pose_y = 0.0
        self.marker_2d_theta = 0.0
        # Fork_param
        self.forwardbackpostion = 0.0
        self.updownposition = 0.0
        self.fork_threshold = 0.015
        # other
        self.check_wait_time = 0
        self.is_triggered = False

    def SpinOnce(self):
        (self.robot_2d_pose_x, self.robot_2d_pose_y, self.robot_2d_theta, \
         self.marker_2d_pose_x, self.marker_2d_pose_y, self.marker_2d_theta)=self.Subscriber.SpinOnce()
    def update_fork(self):
        self.forwardbackpostion, self.updownposition = self.Subscriber.SpinOnce_fork()
    
    def fork_updown(self, desired_updownposition):#0~2.7
        self.update_fork()
        if self.updownposition < desired_updownposition - self.fork_threshold:
            self.pub_fork.publish(self.forkmotion.up.value)
            return False
        elif self.updownposition > desired_updownposition + self.fork_threshold:
            self.pub_fork.publish(self.forkmotion.down.value)
            return False
        else:
            self.pub_fork.publish(self.forkmotion.stop.value)
            return True

    def fork_forwardback(self, desired_forwardbackpostion):# 0~0.7
        self.update_fork()
        if self.forwardbackpostion < desired_forwardbackpostion - self.fork_threshold:
            self.pub_fork.publish(self.forkmotion.forward.value)
            return False
        elif self.forwardbackpostion > desired_forwardbackpostion + self.fork_threshold:
            self.pub_fork.publish(self.forkmotion.backword.value)
            return False
        else:
            self.pub_fork.publish(self.forkmotion.stop.value)
            return True
            
    def fnSeqChangingDirection(self, desired_angle):
        self.SpinOnce()
        desired_angle_turn = -1. *  math.atan2(self.marker_2d_pose_y - 0, self.marker_2d_pose_x - 0)
        
        if desired_angle_turn <0:
            desired_angle_turn = desired_angle_turn + math.pi
        else:
            desired_angle_turn = desired_angle_turn - math.pi

        self.cmd_vel.fnTurn(desired_angle_turn)
        
        if abs(desired_angle_turn) < desired_angle  :
            self.cmd_vel.fnStop()
            if self.check_wait_time > 10 :
                self.check_wait_time = 0
                return True
            else:
                self.check_wait_time =self.check_wait_time  +1
        else:
            self.check_wait_time =0
            return False
        
    def fnSeqChangingtheta(self, desired_angle):
        self.SpinOnce()
        desired_angle_turn = -self.marker_2d_theta

        self.cmd_vel.fnTurn(desired_angle_turn)
        
        if abs(desired_angle_turn) < desired_angle  :
            self.cmd_vel.fnStop()
            if self.check_wait_time > 10 :
                self.check_wait_time = 0
                return True
            else:
                self.check_wait_time =self.check_wait_time  +1
        else:
            self.check_wait_time =0
            return False
    def fnSeqMovingNearbyParkingLot(self):
        self.SpinOnce()
        if self.current_nearby_sequence == self.NearbySequence.initial_turn.value:
            if self.is_triggered == False:
                self.is_triggered = True
                self.initial_robot_pose_theta = self.robot_2d_theta
                self.initial_robot_pose_x = self.robot_2d_pose_x
                self.initial_robot_pose_y = self.robot_2d_pose_y

                self.initial_marker_pose_theta = self.marker_2d_theta
                self.initial_marker_pose_x = self.marker_2d_pose_x

            if self.initial_marker_pose_theta < 0.0:
                desired_angle_turn = (math.pi / 2.0) + self.initial_marker_pose_theta - (self.robot_2d_theta - self.initial_robot_pose_theta)
            elif self.initial_marker_pose_theta > 0.0:
                desired_angle_turn = -(math.pi / 2.0) + self.initial_marker_pose_theta - (self.robot_2d_theta - self.initial_robot_pose_theta)

            desired_angle_turn = -1. * desired_angle_turn
            self.cmd_vel.fnTurn(desired_angle_turn)

            if abs(desired_angle_turn) < 0.03:
                self.cmd_vel.fnStop()
                if self.check_wait_time >10:
                    self.check_wait_time = 0
                    self.current_nearby_sequence = self.NearbySequence.go_straight.value
                    self.is_triggered = False
                else:
                    self.check_wait_time =self.check_wait_time +1
            elif abs(desired_angle_turn) < 0.045 and self.check_wait_time :
                self.cmd_vel.fnStop()
                if self.check_wait_time > 10:
                    self.check_wait_time = 0
                    self.current_nearby_sequence = self.NearbySequence.go_straight.value
                    self.is_triggered = False
                else:
                    self.check_wait_time =self.check_wait_time +1
            else:
                self.check_wait_time =0    

        elif self.current_nearby_sequence == self.NearbySequence.go_straight.value:
            if self.is_triggered == False:
                self.is_triggered = True
                self.initial_robot_pose_x = self.robot_2d_pose_x
                self.initial_robot_pose_y = self.robot_2d_pose_y

            dist_from_start = self.fnCalcDistPoints(self.initial_robot_pose_x, self.robot_2d_pose_x, self.initial_robot_pose_y, self.robot_2d_pose_y)
            desired_dist = -1* self.initial_marker_pose_x * abs(math.cos((math.pi / 2.) - self.initial_marker_pose_theta))

            remained_dist = desired_dist - dist_from_start 
            if remained_dist < 0  :remained_dist =0

            
            self.cmd_vel.fnGoStraight(desired_dist)

            if abs(remained_dist) < 0.01:
                self.cmd_vel.fnStop()
                self.current_nearby_sequence = self.NearbySequence.turn_right.value
                self.is_triggered = False


        elif self.current_nearby_sequence == self.NearbySequence.turn_right.value:
            if self.is_triggered == False:
                self.is_triggered = True
                self.initial_robot_pose_theta = self.robot_2d_theta

            if self.initial_marker_pose_theta < 0.0:
                desired_angle_turn = (math.pi / 2.0) + (self.robot_2d_theta - self.initial_robot_pose_theta)
            elif self.initial_marker_pose_theta > 0.0:
                desired_angle_turn = -(math.pi / 2.0) + (self.robot_2d_theta - self.initial_robot_pose_theta)
            
            self.cmd_vel.fnTurn(desired_angle_turn)
            if abs(desired_angle_turn) < 0.03:
                self.cmd_vel.fnStop()
                if self.check_wait_time > 20:
                    self.check_wait_time = 0
                    self.current_nearby_sequence = self.NearbySequence.parking.value
                    self.is_triggered = False
                    return True                
                else:
                    self.check_wait_time =self.check_wait_time  +1
            elif abs(desired_angle_turn) < 0.045 and self.check_wait_time:
                self.cmd_vel.fnStop()
                if self.check_wait_time > 20:
                    self.check_wait_time = 0
                    self.current_nearby_sequence = self.NearbySequence.parking.value
                    self.is_triggered = False
                    return True                
                else:
                    self.check_wait_time =self.check_wait_time  +1
            else:
                self.check_wait_time =0    
        return False

    def fnSeqParking(self, parking_dist):
        self.SpinOnce()
        desired_angle_turn = math.atan2(self.marker_2d_pose_y - 0, self.marker_2d_pose_x - 0)

        if  not self.cmd_vel.front:
            if desired_angle_turn <0:
                desired_angle_turn = desired_angle_turn + math.pi
            else:
                desired_angle_turn = desired_angle_turn - math.pi


        self.cmd_vel.fnTrackMarker(-desired_angle_turn)

        if (abs(self.marker_2d_pose_x) < parking_dist)  :
            self.cmd_vel.fnStop()
            if self.check_wait_time > 10:
                self.check_wait_time = 0
                return True
            else:
                self.check_wait_time =self.check_wait_time  +1
        elif (abs(self.marker_2d_pose_x) < parking_dist) and self.check_wait_time:
            self.cmd_vel.fnStop()
            if self.check_wait_time > 10:
                self.check_wait_time = 0
                return True
            else:
                self.check_wait_time =self.check_wait_time  +1
        else:
            self.check_wait_time =0
            return False

    # def fnSeqdecide(self, decide_dist):#decide_dist偏離多少公分要後退
    #     self.dist = abs(self.marker_2d_pose_x*math.cos((np.pi/2)-abs(self.marker_2d_theta)))
    #     if  self.dist < decide_dist:
    #         return True
    #     else:
    #         return False

    def fnseqdead_reckoning(self, dead_reckoning_dist):
        self.SpinOnce()
        if self.is_triggered == False:
            self.is_triggered = True
            self.initial_marker_pose_x = self.marker_2d_pose_x 
        dist = abs(self.initial_marker_pose_x - self.marker_2d_pose_x)
        print("dist", dist)
        if dist > dead_reckoning_dist:
            self.cmd_vel.fnStop()
            self.is_triggered = False
            return True
        else:
            self.cmd_vel.fnGoStraight(dist-self.initial_marker_pose_x)
            return False

    def fnCalcDistPoints(self, x1, x2, y1, y2):
        return math.sqrt((x1 - x2) ** 2. + (y1 - y2) ** 2.)


class cmd_vel():
    def __init__(self):
        self.pub_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size = 1)

        self.front = False

    def cmd_pub(self, twist):
        if not self.front:
            twist.linear.x = -twist.linear.x

        if twist.angular.z > 0.3:
            twist.angular.z =0.3 
        elif twist.angular.z < -0.3:
            twist.angular.z =-0.3 
        if twist.linear.x > 0 and twist.linear.x < 0.01:
            twist.linear.x =0.01
        elif twist.linear.x < 0 and twist.linear.x > -0.01:
            twist.linear.x =-0.01   

        if twist.linear.x > 0 and twist.linear.x > 0.2:
            twist.linear.x =0.2
        elif twist.linear.x < 0 and twist.linear.x < -0.2:
            twist.linear.x =-0.2                     
        if twist.angular.z > 0 and twist.angular.z < 0.1:
            twist.angular.z =0.1
        elif twist.angular.z < 0 and twist.angular.z > -0.1:
            twist.angular.z =-0.1
        self.pub_cmd_vel.publish(twist)

    def fnStop(self):
        twist = Twist()
        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0

        self.cmd_pub(twist)

    def fnTurn(self, theta):
        Kp = 0.5 #1.0
        angular_z = Kp * theta
        

        twist = Twist()
        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        
        twist.angular.z = -angular_z

        self.cmd_pub(twist)

    def fnGoStraight(self,v):
        twist = Twist()
        twist.linear.x = v*0.8
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0

        self.cmd_pub(twist)

    def fnGoBack(self):
        twist = Twist()
        twist.linear.x = -0.12
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0

        self.cmd_pub(twist)

    def fnfork(self,direction):
        twist = Twist()
        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = direction
        twist.angular.z = 0

        self.cmd_pub(twist)


    def fnTrackMarker(self, theta):
        Kp = 1.2

        angular_z = Kp * theta

        twist = Twist()
        twist.linear.x = 0.1
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0

        twist.angular.z = -angular_z *0.16
        self.cmd_pub(twist)

