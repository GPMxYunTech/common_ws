#! /usr/bin/env python3
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

def TopologyMap_client(msg):
    client = actionlib.SimpleActionClient('TopologyMap', forklift_server.msg.TopologyMapAction)
    client.wait_for_server()
    command = forklift_server.msg.PBVSGoal(command=msg)
    print("send ", command)
    client.send_goal(command)
    client.wait_for_result()
    return client.get_result()

if __name__ == '__main__':
    rospy.init_node('ctrl_server')

    command =[
        ['PBVS', 'parking_up'], 
        ['TopologyMap', 'v1']
    ]

    for msg in command:
        if(msg[0] == 'PBVS'):
            print("send ", msg[1])
            result = PBVS_client(msg[1])
            print("result ", result)
        elif(msg[0] == 'TopologyMap'):
            print("send ", msg[1])
            result = TopologyMap_client(msg[1])
            print("result ", result)


  
    
