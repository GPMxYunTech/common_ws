#! /usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import actionlib
import forklift_server.msg
import apriltag_ros.msg

def PBVS_client(msg):
    client = actionlib.SimpleActionClient('PBVS_server', forklift_server.msg.PBVSAction)
    client.wait_for_server()
    command = forklift_server.msg.PBVSGoal(command=msg)
    print("send ", command)
    client.send_goal(command)
    client.wait_for_result()
    return client.get_result()

def TopologyMap_client(msg):
    client = actionlib.SimpleActionClient('TopologyMap_server', forklift_server.msg.TopologyMapAction)
    client.wait_for_server()
    goal = forklift_server.msg.TopologyMapGoal(goal=msg)
    print("send ", goal)
    client.send_goal(goal)
    client.wait_for_result()
    return client.get_result()

def AprilTag_up_client(msg):
    client = actionlib.SimpleActionClient('AprilTag_up_server', apriltag_ros.msg.AprilTagAction)
    client.wait_for_server()
    goal = apriltag_ros.msg.AprilTagGoal(goal=msg)
    print("send ", goal)
    client.send_goal(goal)
    client.wait_for_result()
    return client.get_result()

def AprilTag_down_client(msg):
    client = actionlib.SimpleActionClient('AprilTag_down_server', apriltag_ros.msg.AprilTagAction)
    client.wait_for_server()
    goal = apriltag_ros.msg.AprilTagGoal(goal=msg)
    print("send ", goal)
    client.send_goal(goal)
    client.wait_for_result()
    return client.get_result()

if __name__ == '__main__':
    rospy.init_node('ctrl_server')
    rospy.logwarn("ctrl_server start")
    rospy.logwarn("your command list:\n")
    command = rospy.get_param("/ctrl_server/command") 
    for i in command:
        print(i)

    for msg in command:
        rospy.sleep(1)
        if(msg[0] == 'PBVS'):
            print("send PBVS: ", msg[1])
            if(msg[1] == 'parking_bodycamera' or msg[1] == 'drop_pallet'):
                result = AprilTag_up_client(True)
                print("result ", result)
                result = PBVS_client(msg[1])
                print("result ", result)
                result = AprilTag_up_client(False)
                print("result ", result)

            elif(msg[1] == 'parking_forkcamera' or msg[1] == 'raise_pallet'):
                result = AprilTag_down_client(True)
                print("result ", result)
                result = PBVS_client(msg[1])
                print("result ", result)
                result = AprilTag_down_client(False)
                print("result ", result)

        elif(msg[0] == 'TopologyMap'):
            print("send TopologyMap: ", msg[1])
            result = TopologyMap_client(msg[1])
            print("result ", result)
        else:
            print("error command: ", msg)
            
    rospy.signal_shutdown("finish command list")
  
    
