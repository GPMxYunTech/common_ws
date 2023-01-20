#!/usr/bin/env python3
# import Tkinter as tk

import tkinter as tk

class param():
    marker_2d_pose_x = 0.0
    marker_2d_pose_y = 0.0
    marker_2d_theta = 0.0

class windows(param):
    def __init__(self):
        self.window = tk.Tk()
        self.window.geometry('450x450+1700+560') 
        self.labelmarker_2d_pose_x= tk.Label(self.window, text="", font=('Helvetica', 12), fg='black')
        self.label_marker_2d_pose_x = tk.Label(self.window, text="", font=('Helvetica', 12), fg='black')        

        self.labelmarker_2d_pose_y= tk.Label(self.window, text="", font=('Helvetica', 12), fg='black')
        self.label_marker_2d_pose_y = tk.Label(self.window, text="", font=('Helvetica', 12), fg='black')     

        self.labelmarker_2d_theta= tk.Label(self.window, text="", font=('Helvetica', 12), fg='black')
        self.label_marker_2d_theta = tk.Label(self.window, text="", font=('Helvetica', 12), fg='black') 
        self.update_window()
        self.window.mainloop()
    
    def update_window(self):
        place_marker = 10
        self.labelmarker_2d_pose_x.configure(text="marker_2d_pose_x: ")
        self.labelmarker_2d_pose_x.place(x=0, y=place_marker)
        self.label_marker_2d_pose_x.place(x=200, y=place_marker)
        self.label_marker_2d_pose_x.configure(text=param.marker_2d_pose_x)

        self.labelmarker_2d_pose_y.configure(text="marker_2d_pose_y: ")
        self.labelmarker_2d_pose_y.place(x=0, y=place_marker+30)
        self.label_marker_2d_pose_y.place(x=200, y=place_marker+30)
        self.label_marker_2d_pose_y.configure(text=param.marker_2d_pose_y)

        self.labelmarker_2d_theta.configure(text="marker_2d_theta ")
        self.labelmarker_2d_theta.place(x=0, y=place_marker+60)
        self.label_marker_2d_theta.place(x=200, y=place_marker+60)
        self.label_marker_2d_theta.configure(text=param.marker_2d_theta)
        self.window.after(100, self.update_window)
