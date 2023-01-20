#!/usr/bin/env python

import rospy
import tf
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler
from visualization_msgs.msg import Marker
from math import radians, pi

class MoveBaseSquare():
    def __init__(self):
        rospy.init_node('nav_test', anonymous=False)
        
        rospy.on_shutdown(self.shutdown)
        
        # Subscribe to the move_base action server
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        # Wait 60 seconds for the action server to become available
        self.move_base.wait_for_server(rospy.Duration(60))
        rospy.loginfo("Connected to move base server")
        rospy.loginfo("Starting navigation test")
        self.base_frame = rospy.get_param('~base_frame', '/base_footprint')

        # The odom frame is usually just /odom
        self.odom_frame = rospy.get_param('~odom_frame', '/odom')
        goal = MoveBaseGoal()
        quaternion = Quaternion()
        
        self.tf_listener = tf.TransformListener()
        rospy.sleep(2)
        self.tf_listener.waitForTransform(self.odom_frame, self.base_frame, rospy.Time(), rospy.Duration(60.0))        


        self.position = Point()
        self.position = self.get_position()
        #while 1: 
        #goal 1
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        quaternion.x = 0.0
        quaternion.y = 0.0
        quaternion.z = 0.707
        quaternion.w = 0.707
        goal.target_pose.pose.position.x = self.position.x 
        goal.target_pose.pose.position.y = self.position.y
        goal.target_pose.pose.position.z = 0
        goal.target_pose.pose.orientation = quaternion
        
        self.move_base.send_goal(goal)
        
        # Allow 1 minute to get there
        finished_within_time = self.move_base.wait_for_result(rospy.Duration(400)) 
        # If we don't get there in time, abort the goal
        if not finished_within_time:
            self.move_base.cancel_goal()
            rospy.loginfo("Timed out achieving goal")
        else:
            state = self.move_base.get_state()
            if state == GoalStatus.SUCCEEDED:
                rospy.loginfo("Goal succeeded!")

        rospy.sleep(5)
            
    def get_position(self):
        # Get the current transform between the odom and base frames
        try:
            (trans, rot)  = self.tf_listener.lookupTransform(self.odom_frame, self.base_frame, rospy.Time(0))
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.loginfo("TF Exception")
            return

        return Point(*trans)        





    def shutdown(self):
        rospy.loginfo("Stopping the robot...")
        self.move_base.cancel_goal()
        rospy.sleep(2)

if __name__ == '__main__':
    try:
        MoveBaseSquare()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
