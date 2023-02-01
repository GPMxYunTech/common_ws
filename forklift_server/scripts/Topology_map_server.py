#! /usr/bin/env python
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
    "v1":{"v2":4},
    "v2":{"v3":2},
    "v3":{"v4":2, "v2":4},
    "v4":{"v3":4}
}
waypoints = {
    "v1": [1.51, -1.61, 0.0, 1.0],
    "v2": [1.51, -1.61, 0.707, 0.707],
    "v3": [1.62, 1.35, 0.707, 0.707],
    "v4": [1.62, 1.35, 0.0, 1.0]
}

class TopologyMap():
    def __init__(self):
        self.start = input("輸入起始點(v1~v...): ")

    def path(self, goal):
        end = self.find_point(goal)
        print("{}到{}的路径:".format(self.start, end))
        self.parent, self.distance=self.dijkstra(graph,self.start)
        path=self.distance_path(graph,self.start,end)
        self.start = end
        return path

    def find_point(self, goal):
        x, y, z, w = goal.goal.pose.position.x, goal.goal.pose.position.y, goal.goal.pose.orientation.z, goal.goal.pose.orientation.w
        for i in waypoints:
            if x == waypoints[i][0] and y == waypoints[i][1] and z == waypoints[i][2] and w == waypoints[i][3]:
                return i

    # 初始化起点的距离  到自身为零 到其他节点为无穷大
    def init_distance(self, graph,s): #传入图像 和起点
        self.distance={s:0}
        for vertex in graph:
            if vertex !=s:
                self.distance[vertex]=math.inf  #除到本身都为无穷大
        return self.distance
    def dijkstra(self,graph,s):
        pqueue=[]     #创建一个队列
        # 先添加一个起点到队列 和后面加入的排序  
        # 此方法把 队列里面的元素按照优先排列 调用heapop时返回优先级最高的   比如数值最小的
        heapq.heappush(pqueue,(0,s))  
        seen=set() #储存出现过的点
        self.parent={s:None}   #标记此节点的上一个节点  此节点为起点 则父节点为None  
        self.distance=self.init_distance(graph,s)

        while (len(pqueue)>0):
            pair=heapq.heappop(pqueue)  #返回一个数值最小的元组 
            dist=pair[0]  #提取距离
            vertex=pair[1] #提取节点
            seen.add(vertex) #添加出现过的节点
            nodes=graph[vertex].keys()   #提取与vertex相连的节点
            # print(nodes)
            #核心算法
            for w in nodes:
                if w not in seen:
                    if dist+graph[vertex][w]<self.distance[w]:
                        #把路径短的添加到队列 并排序
                        heapq.heappush(pqueue,(dist+graph[vertex][w],w))
                        self.parent[w]=vertex  #记录父节点
                        self.distance[w]=dist+graph[vertex][w] #更新起点到w节点的距离
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
        rz, rw = msg.pose.pose.orientation.z, msg.pose.pose.orientation.w
        yaw_r = math.atan2(2 * rw * rz, rw * rw - rz * rz)
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

    def self_spin(self, z1, w1, z2, w2):
        # rospy.INFO('self_spin')
        self.trigger = False       
        self.odom_pass = 0.0
        rospy.sleep(0.1)

        yaw_1 = math.atan2(2 * w1 * z1, w1 * w1 - z1 * z1)
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
                speed.angular.z = 0.3
            elif(desire_angle <= 0):
                speed.angular.z = -0.3
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
        
        path = self.TopologyMap.path(msg)
        print(path)
        for i in range(len(path)):
            
            if(i > 0 and (waypoints[path[i]][0] == waypoints[path[i-1]][0] and waypoints[path[i]][1] == waypoints[path[i-1]][1])):
                rospy.loginfo('self_spin from %s to %s' % (path[i-1], path[i]))
                rospy.loginfo('self_spin from %s to %s' % (path[i-1], path[i]))
                self.Navigation.self_spin(waypoints[path[i-1]][2], waypoints[path[i-1]][3], waypoints[path[i]][2], waypoints[path[i]][3])
                i = i + 1
                continue
            else:
                rospy.loginfo('Navigation to %s' % path[i])
                self.Navigation.move(waypoints[path[i]][0], waypoints[path[i]][1], waypoints[path[i]][2], waypoints[path[i]][3])

        
        rospy.logwarn('PBVS Succeeded')
        self._result.result = 'success'
        self._as.set_succeeded(self._result)

if __name__ == '__main__':
    rospy.init_node('TopologyMap_server')
    server = TopologyMapAction(rospy.get_name())
    rospy.spin()
