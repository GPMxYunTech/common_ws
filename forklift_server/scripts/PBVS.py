# -*- coding: utf-8 -*-
import rospy
import forklift_server.msg
from enum import Enum
import math
import tkinter as tk
from PBVS_Action import Action
class PBVS():

    
    ParkingSequence = Enum( 'ParkingSequence', \
                            'init_fork \
                            changing_direction_1 \
                            Changingtheta \
                            decide \
                            back \
                            moving_nearby_parking_lot \
                            parking \
                            up_fork_dead_reckoning \
                            up_fork_init \
                            up_fork_up \
                            up_fork_down \
                            up_fork_forward \
                            up_fork_backword \
                            up_fork_tilt_forward \
                            up_fork_tilt_backword \
                            up_fork_back \
                            up_fork_going \
                            down_fork_dead_reckoning \
                            down_fork_init \
                            down_fork_up \
                            down_fork_down \
                            down_fork_forward \
                            down_fork_backword \
                            down_fork_tilt_forward \
                            down_fork_tilt_backword \
                            down_fork_back \
                            down_fork_going \
                            stop')
    

    def __init__(self, _as, Subscriber, Sequence, init_fork, Parking_distance):

        self._as = _as
        self._feedback = forklift_server.msg.PBVSFeedback()
        self._result = forklift_server.msg.PBVSResult()
        self.Subscriber = Subscriber
        self.Sequence = Sequence
        self.init_fork = init_fork
        self.init_PBVS_parame()
        self.Parking_distance = Parking_distance # meter
        self.Action = Action(self.Subscriber)
        self.windows()

    def init_PBVS_parame(self):
        self.is_sequence_finished = False
        self.current_parking_sequence = self.Sequence
        
    def __del__(self):
        rospy.logwarn('delete PBVS')
        # self._result.result = 'success'
        # self._as.set_succeeded(self._result)
     
    def PBVS(self):
        self._feedback.feedback = str(self.ParkingSequence(self.current_parking_sequence))
        self._as.publish_feedback(self._feedback)
        # ============parking============
        if self.current_parking_sequence == self.ParkingSequence.init_fork.value:
            self.is_sequence_finished = self.Action.fork_updown(self.init_fork)
            
            if self.is_sequence_finished == True:
                self.current_parking_sequence = self.ParkingSequence.changing_direction_1.value
                self.is_sequence_finished = False
        elif self.current_parking_sequence == self.ParkingSequence.changing_direction_1.value:
            self.is_sequence_finished = self.Action.fnSeqChangingDirection(0.02)
            
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
                self.current_parking_sequence = self.ParkingSequence.Changingtheta.value
                self.is_sequence_finished = False

        elif self.current_parking_sequence == self.ParkingSequence.Changingtheta.value:
            self.is_sequence_finished = self.Action.fnSeqChangingtheta(0.07)
            
            if self.is_sequence_finished == True:
                self.current_parking_sequence = self.ParkingSequence.decide.value
                self.is_sequence_finished = False

        elif self.current_parking_sequence == self.ParkingSequence.decide.value:
            self.is_sequence_finished = self.Action.fnSeqdecide(0.04)
            
            if self.is_sequence_finished == True:
                self.current_parking_sequence = self.ParkingSequence.stop.value
                self.is_sequence_finished = False

            elif self.is_sequence_finished == False:
                self.current_parking_sequence = self.ParkingSequence.back.value
                self.is_sequence_finished = False

        elif self.current_parking_sequence == self.ParkingSequence.back.value:
            self.is_sequence_finished = self.Action.fnseqdead_reckoning(1.5)
            
            if self.is_sequence_finished == True:
                self.current_parking_sequence = self.ParkingSequence.parking.value
                self.is_sequence_finished = False
        # ============up_fork============

        elif self.current_parking_sequence == self.ParkingSequence.up_fork_init.value:
            self.is_sequence_finished = self.Action.fork_updown(self.init_fork)
            
            if self.is_sequence_finished == True:
                rospy.sleep(0.05)
                self.current_parking_sequence = self.ParkingSequence.up_fork_dead_reckoning.value
                self.is_sequence_finished = False

        elif self.current_parking_sequence == self.ParkingSequence.up_fork_dead_reckoning.value:
            self.is_sequence_finished = self.Action.fnseqdead_reckoning(-0.4)
            
            if self.is_sequence_finished == True:
                rospy.sleep(0.05)
                self.current_parking_sequence = self.ParkingSequence.up_fork_forward.value
                self.is_sequence_finished = False

        elif self.current_parking_sequence == self.ParkingSequence.up_fork_forward.value:
            self.is_sequence_finished = self.Action.fork_forwardback(0.65)
            
            if self.is_sequence_finished == True:
                rospy.sleep(0.05)
                self.current_parking_sequence = self.ParkingSequence.up_fork_up.value
                self.is_sequence_finished = False

        elif self.current_parking_sequence == self.ParkingSequence.up_fork_up.value:
            self.is_sequence_finished = self.Action.fork_updown(0.57)
            
            if self.is_sequence_finished == True:
                rospy.sleep(0.05)
                self.current_parking_sequence = self.ParkingSequence.up_fork_backword.value
                self.is_sequence_finished = False

        elif self.current_parking_sequence == self.ParkingSequence.up_fork_backword.value:
            self.is_sequence_finished = self.Action.fork_forwardback(0)
            
            if self.is_sequence_finished == True:
                rospy.sleep(0.05)
                self.current_parking_sequence = self.ParkingSequence.up_fork_back.value
                self.is_sequence_finished = False
                
        elif self.current_parking_sequence == self.ParkingSequence.up_fork_back.value:
            self.is_sequence_finished = self.Action.fnseqdead_reckoning(1.0)
            
            if self.is_sequence_finished == True:
                rospy.sleep(0.05)
                self.current_parking_sequence = self.ParkingSequence.up_fork_going.value
                self.is_sequence_finished = False

        elif self.current_parking_sequence == self.ParkingSequence.up_fork_going.value:
            self.is_sequence_finished = self.Action.fork_updown(0.392)
            
            if self.is_sequence_finished == True:
                rospy.sleep(0.05)
                self.current_parking_sequence = self.ParkingSequence.stop.value
                self.is_sequence_finished = False
        # ============down_fork============
        elif self.current_parking_sequence == self.ParkingSequence.down_fork_init.value:
            self.is_sequence_finished = self.Action.fork_updown(self.init_fork)
            
            if self.is_sequence_finished == True:
                rospy.sleep(0.05)
                self.current_parking_sequence = self.ParkingSequence.down_fork_dead_reckoning.value
                self.is_sequence_finished = False

        elif self.current_parking_sequence == self.ParkingSequence.down_fork_dead_reckoning.value:
            self.is_sequence_finished = self.Action.fnseqdead_reckoning(-0.4)
            
            if self.is_sequence_finished == True:
                rospy.sleep(0.05)
                self.current_parking_sequence = self.ParkingSequence.down_fork_forward.value
                self.is_sequence_finished = False

        elif self.current_parking_sequence == self.ParkingSequence.down_fork_forward.value:
            self.is_sequence_finished = self.Action.fork_forwardback(0.69)
            
            if self.is_sequence_finished == True:
                rospy.sleep(0.05)
                self.current_parking_sequence = self.ParkingSequence.down_fork_down.value
                self.is_sequence_finished = False

        elif self.current_parking_sequence == self.ParkingSequence.down_fork_down.value:
            self.is_sequence_finished = self.Action.fork_updown(0.67)
            
            if self.is_sequence_finished == True:
                rospy.sleep(0.05)
                self.current_parking_sequence = self.ParkingSequence.down_fork_backword.value
                self.is_sequence_finished = False

        elif self.current_parking_sequence == self.ParkingSequence.down_fork_backword.value:
            self.is_sequence_finished = self.Action.fork_forwardback(0.0)
            
            if self.is_sequence_finished == True:
                rospy.sleep(0.05)
                self.current_parking_sequence = self.ParkingSequence.down_fork_back.value
                self.is_sequence_finished = False

        elif self.current_parking_sequence == self.ParkingSequence.down_fork_back.value:
            self.is_sequence_finished = self.Action.fnseqdead_reckoning(1.0)
            
            if self.is_sequence_finished == True:
                rospy.sleep(0.05)
                self.current_parking_sequence = self.ParkingSequence.down_fork_going.value
                self.is_sequence_finished = False

        elif self.current_parking_sequence == self.ParkingSequence.down_fork_going.value:
            self.is_sequence_finished = self.Action.fork_updown(0.07)
            
            if self.is_sequence_finished == True:
                rospy.sleep(0.05)
                self.current_parking_sequence = self.ParkingSequence.stop.value
                self.is_sequence_finished = False
        # ============stop============
        elif self.current_parking_sequence == self.ParkingSequence.stop.value:

            rospy.logwarn('PBVS Succeeded')
            self.window.destroy()
            rospy.sleep(1)
            
            

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

        # if self.current_parking_sequence == self.ParkingSequence.changing_direction_1.value:
        #     sequence = "changing_direction"
        # elif self.current_parking_sequence == self.ParkingSequence.changing_direction_2.value:
        #     sequence = "changing_direction"
        # elif self.current_parking_sequence == self.ParkingSequence.moving_nearby_parking_lot.value:
        #     sequence = "moving_nearby_parking_lot"
        # elif self.current_parking_sequence == self.ParkingSequence.parking.value:
        #     sequence = "parking"


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

            # base3 = base2+100
            # self.labelfork_pose.configure(text="Fork pose: ")
            # self.labelfork_pose.place(x=0, y=base3)
            # self.label_fork_pose.place(x=200, y=base3)
        except:
            pass
        # self.label_fork_pose.configure(text=self.fork_pose)
        self.window.after(100, self.update_window)
