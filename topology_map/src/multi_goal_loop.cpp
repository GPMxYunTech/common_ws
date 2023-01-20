#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_datatypes.h>
#include "nav_msgs/Odometry.h"
#include <signal.h>
#include <math.h>
#include <cmath>
#include <vector>
using namespace std;
float pre_odom, odom_pass;
bool triggeer = true;
void odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
  float rz, rw;
  rz = msg->pose.pose.orientation.z;
  rw = msg->pose.pose.orientation.w;
  float yaw_r = atan2(2 * rw * rz, rw * rw - rz * rz);
  if (yaw_r < 0)
    yaw_r = yaw_r + 2 * M_PI;
  if (triggeer == false)
  {
    pre_odom = yaw_r;
    odom_pass = 0;
    triggeer = true;
  }
  if (abs(yaw_r - pre_odom) > 1)
    odom_pass = odom_pass;
  else
    odom_pass += yaw_r - pre_odom;
  pre_odom = yaw_r;
}

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char **argv)
{
  // initialize ROS and the node
  ros::init(argc, argv, "multi_goal_loop");
  ros::NodeHandle nh;
  ros::NodeHandle nh_priv("~");
  ros::Rate r(100);
  ros::Duration d(0.1);
  ros::Subscriber odom_sub = nh.subscribe("/odom", 1000, odomCallback);
  ros::Publisher cmdVelPub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  MoveBaseClient ac("move_base", true);

  // configuring parameters
  std::vector<float> goal_list;
  nh_priv.param("goal_list", goal_list, std::vector<float>());
  float turn_angle, yaw_1, yaw_2, w1, z1, w2, z2, yaw_1k;

  while (!ac.waitForServer(ros::Duration(5.0)))
  {
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  int goalSize = goal_list.size() / 4;
  ROS_INFO("goalSize=%d", goalSize);
  int goalFlag = 0;

  for (int i = 0; i < goalSize; i++)
    ROS_INFO("Goal list %d = [%.3f, %.3f, %.3f, %.3f]", i, goal_list[i * 4 + 0], goal_list[i * 4 + 1], goal_list[i * 4 + 2], goal_list[i * 4 + 3]);

  move_base_msgs::MoveBaseGoal goal;
  while (ros::ok())
  {
    for (int i = 0; i < goalSize; i++)
    {
      ROS_INFO("navigation Goal list %d = [%.3f, %.3f, %.3f, %.3f]", i, goal_list[i * 4 + 0], goal_list[i * 4 + 1], goal_list[i * 4 + 2], goal_list[i * 4 + 3]);
      if (goal_list[i * 4 + 0] == goal_list[(i - 1) * 4 + 0] && goal_list[i * 4 + 1] == goal_list[(i - 1) * 4 + 1])
      {
        ROS_INFO("spin\n");
        triggeer = false;
        odom_pass = 0;
        ros::Duration(0.1).sleep();
        z1 = goal_list[(i - 1) * 4 + 2];
        w1 = goal_list[(i - 1) * 4 + 3];
        z2 = goal_list[i * 4 + 2];
        w2 = goal_list[i * 4 + 3];
        yaw_1 = atan2(2 * w1 * z1, w1 * w1 - z1 * z1);
        if (yaw_1 < 0)
          yaw_1 = yaw_1 + 2 * M_PI;
        std::cout << "\nyaw_1 = " << yaw_1 * 180 / M_PI << "\n";
        yaw_2 = atan2(2 * w2 * z2, w2 * w2 - z2 * z2);
        if (yaw_2 < 0)
          yaw_2 = yaw_2 + 2 * M_PI;
        std::cout << "\nyall_2 = " << yaw_2 * 180 / M_PI << "\n";
        
        float desire_angle = yaw_2 - yaw_1;
        if (desire_angle > M_PI)
          desire_angle = desire_angle - 2 * M_PI;
        else if (desire_angle < -M_PI)
          desire_angle = desire_angle + 2 * M_PI;
        cout << "desire_angel  " << desire_angle * 180 / M_PI << endl;
        
        geometry_msgs::Twist speed;
        while (abs(odom_pass) < abs(desire_angle))
        {
          cout << "odom_pass" << odom_pass * 180 / M_PI << endl;

          speed.linear.x = 0;
          if (desire_angle >= 0)
            speed.angular.z = 0.3;
          else if (desire_angle <= 0)
            speed.angular.z = -0.3;

          cmdVelPub.publish(speed);
          ros::Duration(0.01).sleep();
          ros::spinOnce();
        }
        cout << "odom_pass  " << odom_pass * 180 / M_PI << endl;
        cmdVelPub.publish(geometry_msgs::Twist());
        triggeer = true;
        i++;
      }
      goal.target_pose.header.frame_id = "map";
      goal.target_pose.header.stamp = ros::Time::now();
      goal.target_pose.pose.position.x = goal_list[i * 4 + 0];    // goals[i][0];//goal_list[i*4+0]
      goal.target_pose.pose.position.y = goal_list[i * 4 + 1];    // goals[i][1];
      goal.target_pose.pose.orientation.z = goal_list[i * 4 + 2]; // goals[i][2];
      goal.target_pose.pose.orientation.w = goal_list[i * 4 + 3]; // goals[i][3];
      // goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
      ac.sendGoal(goal);
      ROS_INFO("Goal %d set.", i);
      ac.waitForResult();
      if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
      {
        ROS_INFO("Goal %d reached.", i);
        goalFlag = 1;
      }
      else
      {
        ROS_INFO("Goal %d failed.", i);
        goalFlag = 0;
      }
    }
    break; // just 1 round
  }

  return EXIT_SUCCESS;
}
