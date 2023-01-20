#!/usr/bin/env python
# -*- coding: utf-8 -*-
import tkinter as tk
import rospy
import time
from geometry_msgs.msg import PoseStamped

def show():
    a.set(b.get())
    # mypose=PoseStamped()
    # turtle_vel_pub.publish(mypose) 
    # # time.sleep(1)
    mypose=PoseStamped()
    mypose.header.frame_id='base_link'
    mypose.pose.position.x=1.0
    mypose.pose.position.y=2.0
    mypose.pose.position.z=0.0
    mypose.pose.orientation.x=0.0
    mypose.pose.orientation.y=0.0
    mypose.pose.orientation.z=3.0
    mypose.pose.orientation.w=4.0
    
    turtle_vel_pub.publish(mypose)

    # time.sleep(1)


def clear():
    b.set('')
    entry.delete(0,'end')


if __name__ == '__main__':
    rospy.init_node('pubpose')
    turtle_vel_pub = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=1)
    win = tk.Tk()        


    win.title("tkinter test")


    win.geometry("400x400")


    win.maxsize(400,400)
    win.minsize(400,400)

    a = tk.StringVar()
    b = tk.StringVar()
    a.set('')
    
    goallabel = tk.Label(win, text = "goal point")
    goallabel.grid(row =0,column=0)

    entry = tk.Entry(win, textvariable=b)
    entry.grid(row =0,column=1)

    btn1 = tk.Button(win, text='顯示', command=show)   
    btn1.grid(row =0,column=2)
    btn2 = tk.Button(win, text='清除', command=clear)  
    btn2.grid(row =0,column=3)


    showlabel = tk.Label(win, text = "show")
    showlabel.grid(row =1,column=0)

    answer = tk.Label(win, textvariable=a)
    answer.grid(row =1,column=1)

    # input.set('')
    # input.set=b.get()
    win.mainloop()

    
    
