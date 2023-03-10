#! /usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import forklift_server.msg
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import heapq
import math

graph={
    # Visual Servoing Navigation
    "LD1":{"LD2":4},
    "LD2":{"LD3":4},
    "LD3":{"LD4":4},
    "LD4":{"LD5":4},
    "LD5":{"LD6":4},
    "LD6":{"LD7":4},
    "LD7":{"LD8":4},
    "LD8":{"LoopPlace":4},
    # "LD9":{"LD10":4},
    # "LD10":{"LD11":4},
    # "LD11":{"LD12":4},
    # "LD12":{"LoopPlace":4},
    
    "LoopPlace": {"Pick1":1, "Place0":16},
    "Pick1": {"Pick2":2},
    "Pick2": {"Pick3Before":1},
    "Pick3Before": {"Pick3After":1},
    "Pick3After": {"Pick4":1},
    "Pick4": {"Pick5":4},
    "Pick5": {"LoopPlace":4},
    "Place0": {"Place1":4},
    "Place1": {"Place2":4},
    "Place2": {"Place3Before":4},
    "Place3Before": {"Place4":4},
    "Place4": {"Place5":4},
    "Place5": {"LoopPick":4},
    "LoopPick": {"Pick0":4},
    "Pick0": {"Pick1":4},

    # Square Navigation
    "s1": {"s2":4},
    "s2": {"s3":4},
    "s3": {"s4":4},
    "s4": {"s5":4},
    "s5": {"s6":4},
    "s6": {"s7":4},
    "s7": {"s8":4},
    "s8": {"s9":4},
    "s9": {"s1":4, "s8":1}
}
waypoints = {
    "LD1": [9.713,-0.352,0.000,1.000],
    "LD2": [9.713,-0.352,0.649,0.761],
    "LD3": [10.318,7.797,0.707,0.707],
    "LD4": [10.489,13.161,0.705,0.710],
    "LD5": [10.489,13.161,0.143,1.000],
    "LD6": [13.154,13.272,0.000,1.000],
    "LD7": [13.154,13.272,0.659,0.752],
    "LD8": [13.282,19.007,0.707,0.707],
    "LD9": [13.152,29.848,0.688,0.725],
    # "v9": [13.133,24.464,0.697,0.717],
    # "LD10": [13.133,24.464,1.000,0.000],
    # "LD11": [12.098, 24.788,1.000,0.000],
    # "LD12": [12.098, 24.788,0.707,0.707],
    # "v13": []
    "LoopPlace": [12.228,34.329,0.707,0.707],
    "Pick1": [12.228,34.329,0,1],
    "Pick2": [13.882,34.377,0,1],
    "Pick3Before": [13.882,34.377,-0.707,0.707],
    "Pick3After": [14.274,35.536,-0.707,0.707],
    "Pick4": [14.274,35.536,1,0.01],
    "Pick5": [12.228,34.329,1,0.01],
    "Place0": [11.875,47.021,0.696,0.718],
    "Place1": [11.875,47.021,0,1],
    "Place2": [13.896,47.083,0,1],
    "Place3Before": [13.896,47.083,0.697,0.717],
    # "Place3After": [,0.697,0.717],
    "Place4": [13.896,47.083,1,0],
    "Place5": [11.875,47.021,1,0],
    "LoopPick": [11.875,47.021,-0.707,0.707],
    "Pick0": [12.228,34.329,-0.707,0.707],
    "s1": [6.525,-0.220,-0.010,1.000],
    "s2": [6.525,-0.220,0.703,0.711,],
    "s3": [6.440,6.231,0.705,0.709],
    "s4": [6.440,6.231,1.000,0.004],
    "s5": [-5.555,6.011,1.000,0.004],
    "s6": [-5.555,6.011,-0.709,0.705],
    "s7": [-5.540,-0.460,-0.707,0.707],
    "s8": [-5.540,-0.460,0.012,1.000],
    "s9": [0.100,-0.293,0.000,1.000]

}

class TopologyMap():
    def __init__(self):
        self.start = input("???????????????(v1~v...): ")

    def path(self, goal):
        # end = self.find_point(goal)
        print("{}???{}?????????:".format(self.start, goal))
        self.parent, self.distance=self.dijkstra(graph,self.start)
        path=self.distance_path(graph,self.start,goal)
        self.start = goal
        return path

    # def find_point(self, goal):
    #     x, y, z, w = goal.goal.pose.position.x, goal.goal.pose.position.y, goal.goal.pose.orientation.z, goal.goal.pose.orientation.w
    #     for i in waypoints:
    #         if x == waypoints[i][0] and y == waypoints[i][1] and z == waypoints[i][2] and w == waypoints[i][3]:
    #             return i

    # ????????????????????????  ??????????????? ???????????????????????????
    def init_distance(self, graph,s): #???????????? ?????????
        self.distance={s:0}
        for vertex in graph:
            if vertex !=s:
                self.distance[vertex]=math.inf  #???????????????????????????
        return self.distance
    def dijkstra(self,graph,s):
        pqueue=[]     #??????????????????
        # ?????????????????????????????? ????????????????????????  
        # ???????????? ??????????????????????????????????????? ??????heapop???????????????????????????   ?????????????????????
        heapq.heappush(pqueue,(0,s))  
        seen=set() #?????????????????????
        self.parent={s:None}   #?????????????????????????????????  ?????????????????? ???????????????None  
        self.distance=self.init_distance(graph,s)

        while (len(pqueue)>0):
            pair=heapq.heappop(pqueue)  #????????????????????????????????? 
            dist=pair[0]  #????????????
            vertex=pair[1] #????????????
            seen.add(vertex) #????????????????????????
            nodes=graph[vertex].keys()   #?????????vertex???????????????
            # print(nodes)
            #????????????
            for w in nodes:
                if w not in seen:
                    if dist+graph[vertex][w]<self.distance[w]:
                        #?????????????????????????????? ?????????
                        heapq.heappush(pqueue,(dist+graph[vertex][w],w))
                        self.parent[w]=vertex  #???????????????
                        self.distance[w]=dist+graph[vertex][w] #???????????????w???????????????
        return self.parent,self.distance
    def distance_path(self,graph,s,end):
        self.parent, self.distance = self.dijkstra(graph, s)
        path=[end]  
        while self.parent[end] !=None:
            path.append(self.parent[end])
            end=self.parent[end]
        path.reverse()  
        return path

class Navigation():
    def __init__(self):
        self.client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        self.client.wait_for_server()
        self.sub_odom_robot = rospy.Subscriber('/odom', Odometry, self.cbGetRobotOdom, queue_size = 1)
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 1)
        self.init_param()

    def move(self, x, y, z, w):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.orientation.z = z
        goal.target_pose.pose.orientation.w = w

        self.client.send_goal(goal)
        print("Navigation to", goal)
        wait = self.client.wait_for_result()

        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
        else:
            return self.client.get_result()

    def init_param(self):    
        self.trigger = True
        self.pre_odom = 0.0
        self.odom_pass = 0.0
    
    def cbGetRobotOdom(self, msg):
        self.rz, self.rw = msg.pose.pose.orientation.z, msg.pose.pose.orientation.w
        yaw_r = math.atan2(2 * self.rw * self.rz, self.rw * self.rw - self.rz * self.rz)
        if(yaw_r < 0):
            yaw_r = yaw_r + 2 * math.pi

        if(self.trigger == False):
            self.pre_odom = yaw_r
            self.odom_pass = 0.0
            self.trigger = True
        if(abs(yaw_r - self.pre_odom) > 1):
            self.odom_pass = self.odom_pass
        else:
            self.odom_pass = self.odom_pass + yaw_r - self.pre_odom
        self.pre_odom = yaw_r

    def self_spin(self, z2, w2):
        # rospy.INFO('self_spin')
        self.trigger = False       
        self.odom_pass = 0.0
        rospy.sleep(0.1)

        yaw_1 = math.atan2(2 * self.rw * self.rz, self.rw * self.rw - self.rz * self.rz)
        if(yaw_1 < 0):
            yaw_1 = yaw_1 + 2 * math.pi
        # print('yaw_1 = ', yaw_1)
        yaw_2 = math.atan2(2 * w2 * z2, w2 * w2 - z2 * z2)
        if(yaw_2 < 0):
            yaw_2 = yaw_2 + 2 * math.pi
        # print('yaw_2 = ', yaw_2)

        desire_angle = yaw_2 - yaw_1
        if(desire_angle > math.pi):
            desire_angle = desire_angle - 2 * math.pi
        elif(desire_angle < -math.pi):
            desire_angle = desire_angle + 2 * math.pi
        # print('desire_angle = ', desire_angle)

        speed = Twist()
        while(abs(self.odom_pass) < abs(desire_angle)):
            # print("odom_pass", self.odom_pass*180/math.pi)
            if(desire_angle >= 0):
                speed.angular.z = (desire_angle-self.odom_pass)*0.5
            elif(desire_angle <= 0):
                speed.angular.z = (desire_angle-self.odom_pass)*0.5

            if speed.angular.z > 0.3:
                speed.angular.z = 0.3
            elif speed.angular.z < -0.3:
                speed.angular.z = -0.3
            elif speed.angular.z > -0.1 and speed.angular.z < 0:
                speed.angular.z = -0.1
            elif speed.angular.z < 0.1 and speed.angular.z > 0:
                speed.angular.z = 0.1
            self.cmd_pub.publish(speed)
            rospy.sleep(0.01)
        

        self.cmd_pub.publish(Twist())
        self.trigger = True


class TopologyMapAction():
    _result = forklift_server.msg.TopologyMapResult()
    _feedback = forklift_server.msg.TopologyMapFeedback()

    def __init__(self, name):
        self._action_name = name
        self.TopologyMap = TopologyMap()
        self.Navigation = Navigation()
        self._as = actionlib.SimpleActionServer(self._action_name, forklift_server.msg.TopologyMapAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()

    def execute_cb(self, msg):
        rospy.loginfo('TopologyMap receive command : %s' % (msg))
        path = self.TopologyMap.path(msg.goal)
        print(path)
        for i in range(len(path)):
            rospy.sleep(1.0)
            if(i > 0 and (waypoints[path[i]][0] == waypoints[path[i-1]][0] and waypoints[path[i]][1] == waypoints[path[i-1]][1])):
                rospy.loginfo('self_spin from %s to %s' % (path[i-1], path[i]))
                # rospy.loginfo('self_spin from %s to %s' % (path[i-1], path[i]))
                self.Navigation.self_spin(waypoints[path[i]][2], waypoints[path[i]][3])
                i = i + 1
                continue
            else:
                rospy.loginfo('Navigation to %s' % path[i])
                self.Navigation.move(waypoints[path[i]][0], waypoints[path[i]][1], waypoints[path[i]][2], waypoints[path[i]][3])

        
        rospy.logwarn('PBVS Succeeded')
        self._result.result = 'success'
        self._as.set_succeeded(self._result)

if __name__ == '__main__':
    rospy.init_node('TopologyMap')
    server = TopologyMapAction(rospy.get_name())
    rospy.spin()
