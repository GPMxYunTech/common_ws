#! /usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import actionlib
import forklift_server.msg

def PBVS_client(msg):
    client = actionlib.SimpleActionClient('PBVS', forklift_server.msg.PBVSAction)
    client.wait_for_server()
    command = forklift_server.msg.PBVSGoal(command=msg)
    print("send ", command)
    client.send_goal(command)
    client.wait_for_result()
    return client.get_result()

if __name__ == '__main__':
    rospy.init_node('ctrl_server')
    command = ["parking_up", "up", "down"]
    for msg in command:
        print("send ", msg)
        result = PBVS_client(msg)
        print("result ", result)
  
    
