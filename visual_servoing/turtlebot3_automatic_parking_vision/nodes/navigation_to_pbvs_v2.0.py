import rospy
from geometry_msgs.msg import PoseStamped
import actionlib
import forklift_server.msg
from std_msgs.msg import String
from std_msgs.msg import Bool
#include <std_msgs/Int8.h>
class navigation_to_pbvs():
    def __init__(self, command):
        self.TopologyMap_client = actionlib.SimpleActionClient('TopologyMap_server', forklift_server.msg.TopologyMapAction)
        self.TopologyMap_client.wait_for_server()
        self.start_node = rospy.Subscriber('wait', Bool, self.cbwait, queue_size = 100)
        self.command_pub = rospy.Publisher('command', String, queue_size=10)
        self.goalreach = False
        self.wait = False
        for i in range(len(command)):
            print(command[i])
            self.pub_goal(command[i][1], command[i][2], command[i][3], command[i][4])
            
            self.command_pub.publish(command[i][5])
            rospy.sleep(1)
            self.command_pub.publish(command[i][5])
            rospy.sleep(1)
            j=6
            while(j<len(command[i])):
                self.command_pub.publish(command[i][j])
                rospy.sleep(1)
                if(j<=6 and command[i][j] == command[i][j+1]):
                    j = j+1
                    continue
                self.wait_wait(command[i][j])
                j = j+1


                
    def pub_goal(self, x, y, z, w):
        pose=PoseStamped()
        pose.header.stamp = rospy.get_rostime()
        pose.header.frame_id='map'
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = z
        pose.pose.orientation.w = w
        print(pose)
        self.goal_pub.publish(pose)
        rospy.sleep(1)
        self.goal_pub.publish(pose)
        rospy.sleep(1)

    def wait_navigation(self):
        while(self.goalreach == False):
            rospy.sleep(1)
        self.goalreach = False

    def wait_wait(self, command):
        print(command)
        while(self.wait == False):
            rospy.sleep(1)
        self.wait = False

    def spin(self):
        rospy.spin()

    def cbstart_node(self, msg):
        print("receive start node")
        self.goalreach = True

    def cbwait(self, msg):
        print("receive wait")
        self.wait = True
        

if __name__ == '__main__':
    rospy.init_node("navigation_to_pbvs")
    command = \
    [\
        [0, 12.395, 0.847, 0.707, 0.707,"print('start pbvs')",    "self.start()",     "self.dead()" ],\
        [0, 14.874, 1.147, 0.707, 0.707,"print('start pbvs')",    "self.start()",     "self.start()",    "self.dead()"    ],\
        [0, 12.575, 0.908, 0.707, 0.707,"print('start pbvs')",    "self.start()",     "self.start()",    "self.dead()"    ],\
        [0, 14.874, 1.147, 0.707, 0.707,"print('start pbvs')",    "self.start()",     "self.start()",    "self.dead()"    ]
    ]
    navigation_to_pbvs = navigation_to_pbvs(command)
    navigation_to_pbvs.spin()
        #    [0, 12.395, 0.847, 0.707, 0.707,"print('start pbvs')",    "self.start()",     "self.dead()" ],\
       # [0, 14.874, 1.147, 0.707, 0.707,"print('start pbvs')",    "self.start()",     "self.start()",    "self.dead()"    ],\
      #  [0, 12.575, 0.908, 0.707, 0.707,"print('start pbvs')",    "self.start()",     "self.start()",    "self.dead()"    ],\
     #   [0, 14.874, 1.147, 0.707, 0.707,"print('start pbvs')",    "self.start()",     "self.start()",    "self.dead()"    ]
    #    [0, 12.395, 0.847, 0.707, 0.707,"print('start pbvs')",    "self.start()",     "self.dead()",      "self.forklayer_1_1()"],\
     #   [0, 14.874, 1.147, 0.707, 0.707,"print('start pbvs')",    "self.start()",     "self.start()",     "self.forklayer_2_2()", "self.dead()", "self.forklayer_2()"],\
      #  [0, 12.395, 0.847, 0.707, 0.707,"print('start pbvs')",    "self.start()",     "self.start()",      "self.forklayer_2()",     "self.dead()",     "self.forklayer_2_2()"],\
       # [0, 14.874, 1.147, 0.707, 0.707,"print('start pbvs')",    "self.start()",     "self.start()",     "self.forklayer_1_1()", "self.dead()", "self.forklayer_1()"]
    
