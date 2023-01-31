#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import forklift_server.msg
from enum import Enum
import math
import tkinter as tk
from PBVS_Action import Action
class PBVS():
    _feedback = forklift_server.msg.PBVSFeedback()
    _result = forklift_server.msg.PBVSResult()
    
    ParkingSequence = Enum( 'ParkingSequence', \
                            'changing_direction_1 \
                            changing_direction_2 \
                            decide \
                            moving_nearby_parking_lot \
                            parking \
                            up_fork_up \
                            up_fork_down \
                            up_fork_forward \
                            up_fork_backword \
                            up_fork_tilt_forward \
                            up_fork_tilt_backword \
                            down_fork_up \
                            down_fork_down \
                            down_fork_forward \
                            down_fork_backword \
                            down_fork_tilt_forward \
                            down_fork_tilt_backword \
                            stop')
    

    def __init__(self, _as, Subscriber, command):
        self._as = _as
        self.Subscriber = Subscriber
        self.command = command
        self.init_PBVS_parame()
        self.Action = Action(self.Subscriber)
        self.windows()

    def init_PBVS_parame(self):
        self.is_sequence_finished = False
        self.current_parking_sequence = self.command
        self.Parking_distance = 0.5 # meter
        self.dead_reckoning_dist = 0.25 # meter
        self.desire_fork = 0.0
    def __del__(self):
        rospy.logwarn('delete PBVS')
     
    def PBVS(self):
        self._feedback.feedback = self.current_parking_sequence
        self._as.publish_feedback(self._feedback)
        # ============parking============
        if self.current_parking_sequence == self.ParkingSequence.changing_direction_1.value:
            self.is_sequence_finished = self.Action.fnSeqChangingDirection()
            
            if self.is_sequence_finished == True:
                self.current_parking_sequence = self.ParkingSequence.moving_nearby_parking_lot.value
                self.is_sequence_finished = False

        elif self.current_parking_sequence == self.ParkingSequence.moving_nearby_parking_lot.value:
            self.is_sequence_finished = self.Action.fnSeqMovingNearbyParkingLot()
            if self.is_sequence_finished == True:
                self.current_parking_sequence = self.ParkingSequence.parking.value
                self.is_sequence_finished = False

        elif self.current_parking_sequence == self.ParkingSequence.parking.value:
            self.is_sequence_finished = self.Action.fnSeqParking(self.Parking_distance)
            
            if self.is_sequence_finished == True:
                self.current_parking_sequence = self.ParkingSequence.changing_direction_2.value
                self.is_sequence_finished = False

        elif self.current_parking_sequence == self.ParkingSequence.changing_direction_2.value:
            self.is_sequence_finished = self.Action.fnSeqChangingDirection()
            
            if self.is_sequence_finished == True:
                self.current_parking_sequence = self.ParkingSequence.stop.value
                self.is_sequence_finished = False
        # ============up_fork============
        elif self.current_parking_sequence == self.ParkingSequence.up_fork_forward.value:
            self.is_sequence_finished = self.Action.fork_forwardback(0.5)
            
            if self.is_sequence_finished == True:
                rospy.sleep(0.05)
                self.current_parking_sequence = self.ParkingSequence.up_fork_up.value
                self.is_sequence_finished = False

        elif self.current_parking_sequence == self.ParkingSequence.up_fork_up.value:
            self.is_sequence_finished = self.Action.fork_updown(1)
            
            if self.is_sequence_finished == True:
                rospy.sleep(0.05)
                self.current_parking_sequence = self.ParkingSequence.up_fork_backword.value
                self.is_sequence_finished = False

        elif self.current_parking_sequence == self.ParkingSequence.up_fork_backword.value:
            self.is_sequence_finished = self.Action.fork_forwardback(0)
            
            if self.is_sequence_finished == True:
                rospy.sleep(0.05)
                self.current_parking_sequence = self.ParkingSequence.stop.value
                self.is_sequence_finished = False
        # ============down_fork============
        elif self.current_parking_sequence == self.ParkingSequence.down_fork_forward.value:
            self.is_sequence_finished = self.Action.fork_forwardback(0.5)
            
            if self.is_sequence_finished == True:
                rospy.sleep(0.05)
                self.current_parking_sequence = self.ParkingSequence.down_fork_down.value
                self.is_sequence_finished = False

        elif self.current_parking_sequence == self.ParkingSequence.down_fork_down.value:
            self.is_sequence_finished = self.Action.fork_updown(0)
            
            if self.is_sequence_finished == True:
                rospy.sleep(0.05)
                self.current_parking_sequence = self.ParkingSequence.down_fork_backword.value
                self.is_sequence_finished = False

        elif self.current_parking_sequence == self.ParkingSequence.down_fork_backword.value:
            self.is_sequence_finished = self.Action.fork_forwardback(0)
            
            if self.is_sequence_finished == True:
                rospy.sleep(0.05)
                self.current_parking_sequence = self.ParkingSequence.stop.value
                self.is_sequence_finished = False
        
        # ============stop============
        elif self.current_parking_sequence == self.ParkingSequence.stop.value:
            self._result.result = 'success'
            self._as.set_succeeded(self._result)
            rospy.logwarn('PBVS Succeeded')
            self.window.destroy()
            
            

    def windows(self):
        self.window = tk.Tk()
        self.window.geometry('450x450+1700+560') 

        self.labelParkingSequence = tk.Label(self.window, text="", font=('Helvetica', 12), fg='black')
        self.label_ParkingSequence = tk.Label(self.window, text="", font=('Helvetica', 12), fg='#19CAAD') 

        self.labelNearbySequence = tk.Label(self.window, text="", font=('Helvetica', 12), fg='black')
        self.label_NearbySequence = tk.Label(self.window, text="", font=('Helvetica', 12), fg='black') 

        self.labelrobot_2d_pose_x = tk.Label(self.window, text="", font=('Helvetica', 12), fg='black')
        self.label_robot_2d_pose_x = tk.Label(self.window, text="", font=('Helvetica', 12), fg='black') 

        self.labelrobot_2d_pose_y= tk.Label(self.window, text="", font=('Helvetica', 12), fg='black')
        self.label_robot_2d_pose_y = tk.Label(self.window, text="", font=('Helvetica', 12), fg='black')  

        self.labelrobot_2d_theta= tk.Label(self.window, text="", font=('Helvetica', 12), fg='black')
        self.label_robot_2d_theta = tk.Label(self.window, text="", font=('Helvetica', 12), fg='black')   

        self.labelmarker_2d_pose_x= tk.Label(self.window, text="", font=('Helvetica', 12), fg='black')
        self.label_marker_2d_pose_x = tk.Label(self.window, text="", font=('Helvetica', 12), fg='black')        

        self.labelmarker_2d_pose_y= tk.Label(self.window, text="", font=('Helvetica', 12), fg='black')
        self.label_marker_2d_pose_y = tk.Label(self.window, text="", font=('Helvetica', 12), fg='black')     

        self.labelmarker_2d_theta= tk.Label(self.window, text="", font=('Helvetica', 12), fg='black')
        self.label_marker_2d_theta = tk.Label(self.window, text="", font=('Helvetica', 12), fg='black') 

        self.labelfork_pose= tk.Label(self.window, text="", font=('Helvetica', 12), fg='black')
        self.label_fork_pose = tk.Label(self.window, text="", font=('Helvetica', 12), fg='black') 
        
        self.update_window()
        self.window.mainloop()

    def update_window(self):
        self.PBVS()

        (robot_2d_pose_x, robot_2d_pose_y, robot_2d_theta, marker_2d_pose_x, marker_2d_pose_y, marker_2d_theta) = self.Subscriber.SpinOnce()
    
        if self.current_parking_sequence == self.ParkingSequence.changing_direction_1.value:
            sequence = "changing_direction"
        elif self.current_parking_sequence == self.ParkingSequence.changing_direction_2.value:
            sequence = "changing_direction"
        elif self.current_parking_sequence == self.ParkingSequence.moving_nearby_parking_lot.value:
            sequence = "moving_nearby_parking_lot"
        elif self.current_parking_sequence == self.ParkingSequence.parking.value:
            sequence = "parking"


        base = 130

        # self.labelParkingSequence.configure(text="ParkingSequence ")
        # self.labelParkingSequence.place(x=0, y=base+30)        
        # self.label_ParkingSequence.configure(text=sequence)
        # self.label_ParkingSequence.place(x=200, y=base+30)
        try:
            base1 = base+70
            self.labelrobot_2d_pose_x.configure(text='Robot 2d Pose x: ')
            self.labelrobot_2d_pose_x.place(x=0, y=base1)        
            self.label_robot_2d_pose_x.configure(text=robot_2d_pose_x)
            self.label_robot_2d_pose_x.place(x=200, y=base1)

            self.labelrobot_2d_pose_y.configure(text="Robot 2d Pose y: ")
            self.labelrobot_2d_pose_y.place(x=0, y=base1+30)
            self.label_robot_2d_pose_y.place(x=200, y=base1+30)
            self.label_robot_2d_pose_y.configure(text=robot_2d_pose_y)

            self.labelrobot_2d_theta.configure(text="Robot 2d theta: ")
            self.labelrobot_2d_theta.place(x=0, y=base1+60)
            self.label_robot_2d_theta.place(x=200, y=base1+60)
            self.label_robot_2d_theta.configure(text=math.degrees(robot_2d_theta))

            base2 = base1+100
            self.labelmarker_2d_pose_x.configure(text="Marker 2d Pose x: ")
            self.labelmarker_2d_pose_x.place(x=0, y=base2)
            self.label_marker_2d_pose_x.place(x=200, y=base2)
            self.label_marker_2d_pose_x.configure(text=marker_2d_pose_x)

            self.labelmarker_2d_pose_y.configure(text="Marker 2d Pose y: ")
            self.labelmarker_2d_pose_y.place(x=0, y=base2+30)
            self.label_marker_2d_pose_y.place(x=200, y=base2+30)
            self.label_marker_2d_pose_y.configure(text=marker_2d_pose_y)

            self.labelmarker_2d_theta.configure(text="Marker 2d theta: ")
            self.labelmarker_2d_theta.place(x=0, y=base2+60)
            self.label_marker_2d_theta.place(x=200, y=base2+60)
            self.label_marker_2d_theta.configure(text=math.degrees(marker_2d_theta))

            base3 = base2+100
            self.labelfork_pose.configure(text="Fork pose: ")
            self.labelfork_pose.place(x=0, y=base3)
            self.label_fork_pose.place(x=200, y=base3)
        except:
            pass
        # self.label_fork_pose.configure(text=self.fork_pose)
        self.window.after(100, self.update_window)
