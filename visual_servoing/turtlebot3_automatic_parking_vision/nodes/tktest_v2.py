#!/usr/bin/env python
# -*- coding: utf-8 -*-
import tkinter as tk
import rospy
from geometry_msgs.msg import PoseStamped

class windows():
    def __init__(self):
        self.goal_pub = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=1)
        self.init_param()

        self.win = tk.Tk()
        self.win.geometry('600x600') 
        self.entry()
        self.button()
        # self.display()
        self.update_window()
        self.win.mainloop()

    def update_window(self):
        self.display()
        self.win.after(1000, self.update_window)

    def init_param(self):
        self.position_x = 0.0
        self.position_y = 0.0
        self.orientation_z = 0.0
        self.orientation_w = 0.0

    def entry(self):
        self.entry_position_x = tk.Entry(self.win)
        self.entry_position_x.place(x = 10, y = 0)

        self.entry_position_y = tk.Entry(self.win)
        self.entry_position_y.place(x = 10, y = 50)

        self.entry_orientation_z = tk.Entry(self.win)
        self.entry_orientation_z.place(x = 10, y = 100)

        self.entry_orientation_w = tk.Entry(self.win)
        self.entry_orientation_w.place(x = 10, y = 150)

    def button(self):
        self.get_goal_button = tk.Button(self.win, text = "Entry Goal", command = self.get_goal)
        self.get_goal_button.place(x=200, y=0)
        self.send_goal_button = tk.Button(self.win, text = "Send Goal", command = self.send_goal)
        self.send_goal_button.place(x=200, y=50)
    
    def display(self):
        self.display_position_x=tk.Label(self.win, text="position_x")
        self.display_position_x.place(x = 0, y = 200)
        self.display_position_x_=tk.Label(self.win, text = self.position_x)
        self.display_position_x_.place(x = 150, y = 200)

        self.display_position_y=tk.Label(self.win, text="position_y")
        self.display_position_y.place(x = 0, y = 250)
        self.display_position_y_=tk.Label(self.win, text = self.position_y)
        self.display_position_y_.place(x = 150, y = 250)

        self.display_orientation_z=tk.Label(self.win, text="orientation_z")
        self.display_orientation_z.place(x = 0, y = 300)
        self.display_orientation_z_=tk.Label(self.win, text=self.orientation_z)
        self.display_orientation_z_.place(x = 150, y = 300)

        self.display_orientation_w=tk.Label(self.win, text="orientation_w")
        self.display_orientation_w.place(x = 0, y = 350)
        self.display_orientation_w_=tk.Label(self.win, text=self.orientation_z)
        self.display_orientation_w_.place(x = 150, y = 350)

    def get_goal(self):
        self.position_x = float(self.entry_position_x.get())
        self.position_y = float(self.entry_position_y.get())
        self.orientation_z = float(self.entry_orientation_z.get())
        self.orientation_w = float(self.entry_orientation_w.get())
        
    def send_goal(self):
        pose=PoseStamped()
        pose.header.stamp = rospy.get_rostime()
        pose.header.frame_id='base_link'
        pose.pose.position.x = self.position_x
        pose.pose.position.y = self.position_y
        pose.pose.position.z = 0.0
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = self.orientation_z
        pose.pose.orientation.w = self.orientation_w
        self.goal_pub.publish(pose)


if __name__ == '__main__':
    rospy.init_node('pubpose')
    windows = windows()
    
    
