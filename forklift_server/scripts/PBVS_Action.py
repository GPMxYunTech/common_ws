# -*- coding: utf-8 -*-
import rospy
import numpy as np
import math
from geometry_msgs.msg import Twist
from enum import Enum
from gpm_msg.msg import forklift
import statistics


class Action():
    def __init__(self, Subscriber):
        # cmd_vel
        self.cmd_vel = cmd_vel()
        self.Subscriber = Subscriber
        self.NearbySequence = Enum(
            'NearbySequence', 'initial_turn go_straight turn_right parking ')
        self.current_nearby_sequence = self.NearbySequence.initial_turn.value
        # fork_cmd
        self.pub_fork = rospy.Publisher('/cmd_fork', forklift, queue_size=1)
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
        self.initial_robot_pose_x = 0.0
        self.initial_robot_pose_y = 0.0
        # AprilTag_param
        self.is_marker_pose_received = False
        self.marker_2d_pose_x = 0.0
        self.marker_2d_pose_y = 0.0
        self.marker_2d_theta = 0.0
        self.initial_marker_pose_x = 0.0
        self.initial_marker_pose_y = 0.0
        self.initial_marker_pose_theta = 0.0
        self.marker_2d_theta_list = [0.0]
        # Fork_param
        self.forwardbackpostion = 0.0
        self.updownposition = 0.0
        self.fork_threshold = 0.01
        # other
        self.check_wait_time = 0
        self.is_triggered = False

    def SpinOnce(self):
        (self.robot_2d_pose_x, self.robot_2d_pose_y, self.robot_2d_theta,
         self.marker_2d_pose_x, self.marker_2d_pose_y, self.marker_2d_theta) = self.Subscriber.SpinOnce()

    def update_fork(self):
        self.forwardbackpostion, self.updownposition = self.Subscriber.SpinOnce_fork()

    def fork_updown_finetune(self, desired_updownposition, fork_threshold):
        self.update_fork()

        while (abs(self.updownposition - desired_updownposition) > fork_threshold):
            if self.updownposition < desired_updownposition - fork_threshold:

                self.pub_fork.publish(self.forkmotion.up.value)
                rospy.sleep(0.2)
                self.pub_fork.publish(self.forkmotion.stop.value)
            elif self.updownposition > desired_updownposition + fork_threshold:

                self.pub_fork.publish(self.forkmotion.down.value)
                rospy.sleep(0.05)
                self.pub_fork.publish(self.forkmotion.stop.value)
            rospy.sleep(0.7)
            self.update_fork()
        print(self.updownposition, desired_updownposition)

    def fork_updown(self, desired_updownposition):  # 新的流程只需要給停止位置
        self.update_fork()
        cmd_fork = forklift()
        cmd_fork.controlmode = 1
        cmd_fork.motionaxis = 1

        if abs(desired_updownposition-self.updownposition) > 0.005:
            cmd_fork.updownposition = desired_updownposition
            self.pub_fork.publish(cmd_fork)
            return False
        else:
            return True

    def fork_forwardback(self, desired_forwardbackpostion):  # 新的流程只需要給停止位置
        self.update_fork()
        cmd_fork = forklift()
        cmd_fork.controlmode = 1
        cmd_fork.motionaxis = 2

        if abs(desired_forwardbackpostion-self.forwardbackpostion) > 0.005:
            cmd_fork.frontrearposition = desired_forwardbackpostion
            self.pub_fork.publish(cmd_fork)
            return False
        else:
            return True

    def fork_updown_old(self, desired_updownposition):  # 0~2.7
        self.update_fork()
        if self.updownposition < desired_updownposition - self.fork_threshold:
            self.pub_fork.publish(self.forkmotion.up.value)
            return False
        elif self.updownposition > desired_updownposition + self.fork_threshold:
            self.pub_fork.publish(self.forkmotion.down.value)
            return False
        else:

            self.fork_updown_finetune(desired_updownposition, 0.005)
            self.pub_fork.publish(self.forkmotion.stop.value)
            return True

    def fork_forwardback_old(self, desired_forwardbackpostion):  # 0~0.7
        self.update_fork()
        if self.forwardbackpostion < desired_forwardbackpostion - self.fork_threshold*1.5:
            self.pub_fork.publish(self.forkmotion.forward.value)
            return False
        elif self.forwardbackpostion > desired_forwardbackpostion + self.fork_threshold*1.5:
            self.pub_fork.publish(self.forkmotion.backword.value)
            return False
        else:
            self.pub_fork.publish(self.forkmotion.stop.value)
            return True

    def fnSeqChangingDirection(self, desired_angle):
        self.SpinOnce()
        desired_angle_turn = -1. * \
            math.atan2(self.marker_2d_pose_y, self.marker_2d_pose_x)

        if desired_angle_turn < 0:
            desired_angle_turn = desired_angle_turn + math.pi
        else:
            desired_angle_turn = desired_angle_turn - math.pi

        self.cmd_vel.fnTurn(desired_angle_turn)

        if abs(desired_angle_turn) < desired_angle:
            self.cmd_vel.fnStop()
            if self.check_wait_time > 10:
                self.check_wait_time = 0
                return True
            else:
                self.check_wait_time = self.check_wait_time + 1
                return False
        else:
            self.check_wait_time = 0
            return False

    def fnSeqChangingtheta(self, threshod):
        #new_version
        #每次使用前先清掉之前的資料
        if self.is_triggered == False:
            self.is_triggered = True
            self.marker_2d_theta_list.clear()

        self.SpinOnce()
        self.marker_2d_theta_list.append(self.marker_2d_theta)
        print("start angle")

        #累計超過20筆後才開始處理
        if(len(self.marker_2d_theta_list)>20):
            #幹掉最舊的一筆
            del self.marker_2d_theta_list[0]

            #濾除超過1倍4分位數的資料
            n = 1
            # IQR = Q3-Q1
            IQR = np.percentile(self.marker_2d_theta_list, 75) - np.percentile(self.marker_2d_theta_list, 25)
            print(IQR)
            # outlier = Q3 + n*IQR
            transform_list = np.array(self.marker_2d_theta_list)[self.marker_2d_theta_list < np.percentile(self.marker_2d_theta_list, 75) + n * IQR]
            # outlier = Q1 - n*IQR
            transform_list = np.array(transform_list)[transform_list > np.percentile(transform_list, 25) - n * IQR]

            #最多只取最後5筆的平均值當角度
            average_theta = 0
            if(len(transform_list)>5):
                average_theta = statistics.mean(transform_list[-5:])
            else:
                average_theta = statistics.mean(transform_list)
            desired_angle_turn = - average_theta
            print("average_theta")
            print(average_theta)

            #根據角度動車
            if abs(desired_angle_turn) < threshod:
                self.cmd_vel.fnStop()

                #等1秒讓車徹底停下來
                rospy.sleep(1.0)

                #確定有轉正後更新左右偏差
                self.SpinOnce()
                self.initial_marker_pose_y = self.marker_2d_pose_y
                print("initial_marker_pose_y")
                print(self.initial_marker_pose_y)
                self.is_triggered = False

                return True
            else:
                self.cmd_vel.fnTurn(desired_angle_turn)
                rospy.sleep(0.05)
                return False
        else:
            rospy.sleep(0.05)
            return False

    def TurnByTime(self, desired_angle_turn, time):
        initial_time = rospy.Time.now().secs
        while (abs(initial_time - rospy.Time.now().secs) < time):
            self.cmd_vel.fnTurn(desired_angle_turn)
            rospy.sleep(0.1)
        self.cmd_vel.fnStop()

    def fnseqturn(self, threshod):  # 旋轉到後退所需角度
        self.SpinOnce()
        if (self.marker_2d_pose_y > 0):
            self.marker_2d_theta = self.marker_2d_theta + 0.15
        else:
            self.marker_2d_theta = self.marker_2d_theta - 0.15

        desired_angle_turn = -self.marker_2d_theta
        if abs(desired_angle_turn) < threshod:
            self.cmd_vel.fnStop()
            rospy.sleep(0.1)
            if self.check_wait_time > 5:
                self.check_wait_time = 0
                return True
            else:
                self.check_wait_time = self.check_wait_time + 1
                return False
        else:
            self.cmd_vel.fnTurn(desired_angle_turn)
            rospy.sleep(0.3)
            self.check_wait_time = 0
            return False

    #決定要不要直角轉彎挪位置
    def fnSeqMovingNearbyParkingLot(self):
        self.SpinOnce()
        if self.current_nearby_sequence == self.NearbySequence.initial_turn.value:
            if self.is_triggered == False:
                self.is_triggered = True
                self.initial_robot_pose_theta = self.robot_2d_theta
                self.initial_robot_pose_x = self.robot_2d_pose_x
                self.initial_robot_pose_y = self.robot_2d_pose_y

                self.initial_marker_pose_theta = self.TrustworthyMarker2DTheta(3)
                self.initial_marker_pose_x = self.marker_2d_pose_x
                print("initial_marker_pose_theta ",
                      self.initial_marker_pose_theta)
                # decide doing fnSeqMovingNearbyParkingLot or not
                
                #用角度計算左右偏差
                #desired_dist = -1* self.initial_marker_pose_x * abs(math.cos((math.pi / 2.) - self.initial_marker_pose_theta))

                # 直接用之前紀錄的左右偏差
                desired_dist = self.initial_marker_pose_y
                
                if abs(desired_dist) < 0.15:
                    self.is_triggered = False
                    return True

            #轉90度
            if self.initial_marker_pose_theta < 0.0:
                desired_angle_turn = (math.pi / 2.0) + self.initial_marker_pose_theta - abs(
                    self.robot_2d_theta - self.initial_robot_pose_theta)
            elif self.initial_marker_pose_theta > 0.0:
                desired_angle_turn = -(math.pi / 2.0) + self.initial_marker_pose_theta + abs(
                    self.robot_2d_theta - self.initial_robot_pose_theta)

            #new version
            desired_angle_turn = -1. * desired_angle_turn
            #角度夠小直接下一動
            if abs(desired_angle_turn) <= 0.02:
                self.cmd_vel.fnStop()
                #切換到直走流程
                self.current_nearby_sequence = self.NearbySequence.go_straight.value
                self.is_triggered = False
            #角度不夠小但轉得有點久也下一動
            elif 0.02 < abs(desired_angle_turn) < 0.06:

                if self.check_wait_time>15:
                    self.cmd_vel.fnStop()
                    # 切換到直走流程
                    self.current_nearby_sequence = self.NearbySequence.go_straight.value
                    self.is_triggered = False
                else:
                    self.cmd_vel.fnTurn(desired_angle_turn)
                    rospy.sleep(0.01)
                    print(desired_angle_turn)
                    self.check_wait_time += 1
            else:
                self.cmd_vel.fnTurn(desired_angle_turn)
                rospy.sleep(0.01)
                print(desired_angle_turn)
                self.check_wait_time=0

            # #old versiom
            # desired_angle_turn = -1. * desired_angle_turn
            # self.cmd_vel.fnTurn(desired_angle_turn)
            #
            # if abs(desired_angle_turn) < 0.03:
            #     self.cmd_vel.fnStop()
            #     if self.check_wait_time > 10:
            #         self.check_wait_time = 1
            #         self.current_nearby_sequence = self.NearbySequence.go_straight.value
            #         self.is_triggered = False
            #     else:
            #         self.check_wait_time = self.check_wait_time + 1
            # elif abs(desired_angle_turn) < 0.045 and self.check_wait_time:
            #     self.cmd_vel.fnStop()
            #     if self.check_wait_time > 10:
            #         self.check_wait_time = 1
            #         self.current_nearby_sequence = self.NearbySequence.go_straight.value
            #         self.is_triggered = False
            #     else:
            #         self.check_wait_time = self.check_wait_time + 1
            # else:
            #     self.check_wait_time = 0

        #轉90度後直走
        elif self.current_nearby_sequence == self.NearbySequence.go_straight.value:
            if self.is_triggered == False:
                self.is_triggered = True
                self.initial_robot_pose_x = self.robot_2d_pose_x
                self.initial_robot_pose_y = self.robot_2d_pose_y
                #加個sleep避免車子甩尾
                rospy.sleep(0.2)

            dist_from_start = self.fnCalcDistPoints(self.initial_robot_pose_x, self.robot_2d_pose_x, self.initial_robot_pose_y, self.robot_2d_pose_y)
            
            # 用角度計算左右偏差
            # desired_dist = -1* self.initial_marker_pose_x * abs(math.cos((math.pi / 2.) - self.initial_marker_pose_theta))

            # 直接用之前紀錄的左右偏差
            desired_dist = abs(self.initial_marker_pose_y)
            

            # HACK: self spin error correct

            desired_dist = desired_dist

            remained_dist = desired_dist - dist_from_start
            if remained_dist < 0:
                remained_dist = 0

            self.cmd_vel.fnGoStraight(desired_dist)

            if abs(remained_dist) < 0.07:
                self.cmd_vel.fnStop()
                self.current_nearby_sequence = self.NearbySequence.turn_right.value
                self.is_triggered = False

        #轉回來
        elif self.current_nearby_sequence == self.NearbySequence.turn_right.value:
            if self.is_triggered == False:
                self.SpinOnce()
                self.is_triggered = True
                self.initial_robot_pose_theta = self.robot_2d_theta
                # 加個sleep避免車子甩尾
                rospy.sleep(0.2)

            if self.initial_marker_pose_theta < 0.0:
                desired_angle_turn = (
                    math.pi / 2.0) - abs(self.robot_2d_theta - self.initial_robot_pose_theta)
            elif self.initial_marker_pose_theta > 0.0:
                desired_angle_turn = - \
                    (math.pi / 2.0) + abs(self.robot_2d_theta -
                                       self.initial_robot_pose_theta)

            # new version
            # 角度夠小直接下一動
            if abs(desired_angle_turn) <= 0.02:
                self.cmd_vel.fnStop()
                # 結束流程
                self.current_nearby_sequence = self.NearbySequence.parking.value
                self.is_triggered = False
                return True
            # 角度不夠小但轉得有點久也下一動
            elif 0.02 < abs(desired_angle_turn) < 0.06:

                if self.check_wait_time > 15:
                    self.cmd_vel.fnStop()
                    # 結束流程
                    self.current_nearby_sequence = self.NearbySequence.parking.value
                    self.is_triggered = False
                    return True
                else:
                    self.cmd_vel.fnTurn(desired_angle_turn)
                    rospy.sleep(0.01)
                    print(desired_angle_turn)
                    self.check_wait_time += 1
            else:
                self.cmd_vel.fnTurn(desired_angle_turn)
                rospy.sleep(0.01)
                print(desired_angle_turn)
                self.check_wait_time = 0

            # # old versiom
            # self.cmd_vel.fnTurn(desired_angle_turn)
            # if abs(desired_angle_turn) < 0.03:
            #     self.cmd_vel.fnStop()
            #     if self.check_wait_time > 20:
            #         self.check_wait_time = 0
            #         self.current_nearby_sequence = self.NearbySequence.parking.value
            #         self.is_triggered = False
            #         return True
            #     else:
            #         self.check_wait_time = self.check_wait_time + 1
            # elif abs(desired_angle_turn) < 0.045 and self.check_wait_time:
            #     self.cmd_vel.fnStop()
            #     if self.check_wait_time > 20:
            #         self.check_wait_time = 0
            #         self.current_nearby_sequence = self.NearbySequence.parking.value
            #         self.is_triggered = False
            #         return True
            #     else:
            #         self.check_wait_time = self.check_wait_time + 1
            # else:
            #     self.check_wait_time = 0
        return False

    def fnSeqParking(self, parking_dist):
        self.SpinOnce()
        desired_angle_turn = math.atan2(
            self.marker_2d_pose_y - 0, self.marker_2d_pose_x - 0)

        if desired_angle_turn < 0:
            desired_angle_turn = desired_angle_turn + math.pi
        else:
            desired_angle_turn = desired_angle_turn - math.pi

        if abs(desired_angle_turn) <= 0.02:
            desired_angle_turn=0
        self.cmd_vel.fnTrackMarker(-desired_angle_turn)

        if (abs(self.marker_2d_pose_x) < parking_dist):
            self.cmd_vel.fnStop()
            if self.check_wait_time > 10:
                self.check_wait_time = 0
                return True
            else:
                self.check_wait_time = self.check_wait_time + 1
        elif (abs(self.marker_2d_pose_x) < parking_dist) and self.check_wait_time:
            selfnseqdead_reckoningf.cmd_vel.fnStop()
            if self.check_wait_time > 10:
                self.check_wait_time = 0
                return True
            else:
                self.check_wait_time = self.check_wait_time + 1
        else:
            self.check_wait_time = 0
            return False

    def fnSeqdecide(self, decide_dist):  # decide_dist偏離多少公分要後退
        self.SpinOnce()
        dist = self.marker_2d_pose_y
        if abs(dist) < abs(decide_dist):
            return True
        else:
            return False

    # (使用里程紀計算)移動到離現在位置dead_reckoning_dist公尺的地方
    def fnseqdead_reckoning(self, dead_reckoning_dist):
        self.SpinOnce()
        if self.is_triggered == False:
            self.is_triggered = True
            self.initial_robot_pose_x = self.robot_2d_pose_x
            self.initial_robot_pose_y = self.robot_2d_pose_y
        dist = math.copysign(1, dead_reckoning_dist) * self.fnCalcDistPoints(
            self.initial_robot_pose_x, self.robot_2d_pose_x, self.initial_robot_pose_y, self.robot_2d_pose_y)
        # print("dist", dist)
        if math.copysign(1, dead_reckoning_dist) > 0.0:
            if dead_reckoning_dist - dist < 0.0:
                self.cmd_vel.fnStop()
                self.is_triggered = False
                return True
            else:
                self.cmd_vel.fnGoStraight(-(dead_reckoning_dist - dist))
                return False
        elif math.copysign(1, dead_reckoning_dist) < 0.0:
            if dead_reckoning_dist - dist > 0.0:
                self.cmd_vel.fnStop()
                self.is_triggered = False
                return True
            else:
                self.cmd_vel.fnGoStraight(-(dead_reckoning_dist - dist))
                return False

    # (使用marker)前後移動到距離marker_dist公尺的位置
    def fnseqmove_to_marker_dist(self, marker_dist):
        self.SpinOnce()
        if (marker_dist < 2.0):
            threshold = 0.015
        else:
            threshold = 0.03

        dist = math.sqrt(self.marker_2d_pose_x**2 + self.marker_2d_pose_y**2)

        if dist < (marker_dist-threshold):
            self.cmd_vel.fnGoStraight(-(marker_dist - dist))
            return False
        elif dist > (marker_dist+threshold):
            self.cmd_vel.fnGoStraight(-(marker_dist - dist))
            return False
        else:
            self.cmd_vel.fnStop()
            return True

    def fnCalcDistPoints(self, x1, x2, y1, y2):
        return math.sqrt((x1 - x2) ** 2. + (y1 - y2) ** 2.)

    def TrustworthyMarker2DTheta(self, time):
        marker_2d_theta_list_observe = [0.0]
        initial_time = rospy.Time.now().secs
        print("self.marker_2d_theta_1", self.marker_2d_theta)
        while (abs(initial_time - rospy.Time.now().secs) < time):
            self.SpinOnce()
            marker_2d_theta_list_observe.append(self.marker_2d_theta)
            # print("self.marker_2d_theta", self.marker_2d_theta)
            rospy.sleep(0.05)
        # print("marker_2d_theta_list_observe", marker_2d_theta_list_observe)
        threshold = 0.5
        mean = statistics.mean(marker_2d_theta_list_observe)
        stdev = statistics.stdev(marker_2d_theta_list_observe)
        upcutoff = mean + threshold * stdev
        downcutoff = mean - threshold * stdev
        clean_list = []
        for i in marker_2d_theta_list_observe:
            if (i > downcutoff and i < upcutoff):
                clean_list.append(i)

        return statistics.median(clean_list)


class cmd_vel():
    def __init__(self):
        self.pub_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.front = False

    def cmd_pub(self, twist):
        if not self.front:
            twist.linear.x = -twist.linear.x

        if twist.angular.z > 0.2:
            twist.angular.z = 0.2
        elif twist.angular.z < -0.2:
            twist.angular.z = -0.2
        if twist.linear.x > 0 and twist.linear.x < 0.04:
            twist.linear.x = 0.04
        elif twist.linear.x < 0 and twist.linear.x > -0.04:
            twist.linear.x = -0.04

        if twist.linear.x > 0.2:
            twist.linear.x = 0.2
        elif twist.linear.x < -0.2:
            twist.linear.x = -0.2
        if twist.angular.z > 0 and twist.angular.z < 0.01:
            twist.angular.z = 0.01
        elif twist.angular.z < 0 and twist.angular.z > -0.01:
            twist.angular.z = -0.01
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
        if abs(theta)<0.17:
            if theta>0:
                theta=0.17
            else:
                theta=-0.17
        Kp = 0.3  # 1.0
        angular_z = Kp * theta

        twist = Twist()
        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0

        twist.angular.z = -angular_z

        self.cmd_pub(twist)

    def fnGoStraight(self, v):
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
        twist.linear.x = -0.1
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0

        self.cmd_pub(twist)

    def fnfork(self, direction):
        twist = Twist()
        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = direction
        twist.angular.z = 0

        self.cmd_pub(twist)

    def fnTrackMarker(self, theta):
        Kp = 6.2  # 6.5

        twist = Twist()
        twist.linear.x = 0.15
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0

        twist.angular.z = -Kp * theta
        self.cmd_pub(twist)
