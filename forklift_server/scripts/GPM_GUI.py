#! /usr/bin/env python3
# -*- coding: utf-8 -*-
import tkinter as tk
from tkinter import *
from tkinter.constants import CENTER
import subprocess
import rospy
from nav_msgs.msg import Odometry
from gpm_msg.msg import agvmotion

from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3


class ScriptExecutor():
    def __init__(self):
        self.init_param()
        self.window = tk.Tk()
        self.window.title("Script Executor")
        self.window.geometry('1024x800') 
        # self.entry()
        self.update_window_once()
        # self.update_window()
        self.button()
        self.window.mainloop()
        
        
        # Subscriber
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.odom_callback)
        self.agvmotion_sub = rospy.Subscriber("/agvmotion", agvmotion, self.agvmotion_callback)
        
    def init_param(self):
        self.test = 0.0
    
    def execute_script_1_1(self):
        subprocess.call(['/home/user/shellscript/3DLiDARProject.sh'])

    def execute_script_1_2(self):
        subprocess.call(['/home/user/shellscript/3DLiDARProject_Demo.sh'])

    def execute_script_2(self):
        subprocess.call(['/home/user/shellscript/VisualProject.sh'])

    def execute_script_3(self):
        subprocess.call(['/home/user/shellscript/AntiRolloverMonitoring.sh'])

    
    def update_window_once(self):
        self.display_once()
    
    # def update_window(self):
    #     self.display()
    #     self.window.after(1000, self.update_window)


    def button(self):
        self.button1_1 = tk.Button(
            self.window, 
            text="3D-LiDAR Navigation + Visual Servoing (Only Servoing)", 
            font=("Times New Romen", 20, "bold"), 
            command=self.execute_script_1_1)
        self.button1_1.pack()

        self.button1_2 = tk.Button(
            self.window, 
            text="3D-LiDAR Navigation + Visual Servoing (Demo Version)", 
            font=("Times New Romen", 20, "bold"), 
            command=self.execute_script_1_2)
        self.button1_2.pack()

        self.button2 = tk.Button(
            self.window, 
            text="3D-Vision Navigation + Visual Servoing (Demo Version)", 
            font=("Times New Romen", 20, "bold"), 
            command=self.execute_script_2)
        self.button2.pack()

        self.button3 = tk.Button(
            self.window, 
            text="Hybrid Fuzzy-Fuzzy Stability Control of Anti-Rollover", 
            font=("Times New Romen", 20, "bold"), 
            command=self.execute_script_3)
        self.button3.pack()
        
    def display_once(self):
        self.title = tk.Label(
            text="GPM Autonomous Forklift", 
            font=("Times New Romen", 36, "bold"), 
            padx=5, 
            pady=5, 
            fg="black")
        self.title.pack()

    # def display(self):
        
    def odometry_callback(self,msg):
        self.odom_position.x = msg.pose.position.x
        self.odom_position.y = msg.pose.position.y
        self.odom_orientation.w = msg.pose.orientation


if __name__ == '__main__':
    rospy.init_node('ScriptExecutor')
    ScriptExecutor = ScriptExecutor()