#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseStamped.h>
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Int8.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <signal.h>
#include <math.h>
#include <cmath>
#include <vector>
using namespace std;
ros::Publisher cmdVelPub;
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
nav_msgs::Odometry amcl;
std_msgs::Int16MultiArray msg;

void array_Callback(const std_msgs::Int16MultiArray::ConstPtr &Msg)
{
  std::cout << "receive path is ";
  for (int i = 0; i < Msg->data.size(); i++)
  {
    std::cout << Msg->data.at(i) << " -> ";
    msg.data.push_back(Msg->data.at(i));
  }
}

void turn(float goal_angle);
int main(int argc, char **argv)
{
  // initialize ROS and the node
  ros::init(argc, argv, "auto_goal_path");
  ros::NodeHandle nh;
  ros::NodeHandle nh_priv("~");
  ros::Rate loop_rate(2);
  ros::Duration d(0.1);
  ros::Subscriber sub = nh.subscribe("path", 2, array_Callback);
  ros::Publisher start_node_pub = nh.advertise<std_msgs::Int8>("start_node", 10);
  move_base_msgs::MoveBaseGoal goal;
  std_msgs::Int8 start;
  float time0, time1;
  cmdVelPub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  MoveBaseClient ac("move_base", true);
  ROS_INFO("**********************************");

  // configuring parameters
  std::vector<float> waypoint, goal_list;
  nh_priv.param("waypoint", waypoint, std::vector<float>());
  float turn_angle, yaw_1, yaw_2, w1, z1, w2, z2, yaw_1k;

  for (int i = 0; i < waypoint.size(); i++)
    // std::cout<<waypoint.at(i)<<" "<<std::endl;
    while (!ac.waitForServer(ros::Duration(5.0)))
    {
      ROS_INFO("Waiting for the move_base action server to come up");
    }

  while (ros::ok())
  {
    
    ros::spinOnce();
    if (msg.data.size() == 0)
    ros::Duration(0.1).sleep();
      continue;

    for (int i = 0; i < msg.data.size(); i++)
    {
      for (int j = 0; j < 4; j++)
      {
        // std::cout<<"push_back waypoint "<<msg.data.at(i)*4+j<<" value "<<waypoint[msg.data.at(i)*4+j]<<std::endl;
        goal_list.push_back(waypoint[msg.data.at(i) * 4 + j]);
      }

      ROS_INFO("Goal list %d = [%.3f, %.3f, %.3f, %.3f]", i, goal_list[i * 4 + 0], goal_list[i * 4 + 1], goal_list[i * 4 + 2], goal_list[i * 4 + 3]);
    }

    int goalSize = goal_list.size() / 4;
    ROS_INFO("goalSize=%d", goalSize);

    for (int i = 0; i < msg.data.size(); i++)
    {
      ROS_INFO("navigation Goal list %d = [%.3f, %.3f, %.3f, %.3f]", i, goal_list[i * 4 + 0], goal_list[i * 4 + 1], goal_list[i * 4 + 2], goal_list[i * 4 + 3]);

      if (goal_list[i * 4 + 0] == goal_list[(i - 1) * 4 + 0] && goal_list[i * 4 + 1] == goal_list[(i - 1) * 4 + 1])
      {
        z1 = goal_list[(i - 1) * 4 + 2];
        w1 = goal_list[(i - 1) * 4 + 3];
        z2 = goal_list[i * 4 + 2];
        w2 = goal_list[i * 4 + 3];
        yaw_1 = atan2(2 * w1 * z1, w1 * w1 - z1 * z1);
        yaw_2 = atan2(2 * w2 * z2, w2 * w2 - z2 * z2);
        // std::cout<<"yaw_1 = "<<yaw_1<<"\nyall_2 = "<<yaw_2<<"\n";
        if (yaw_1 < 0)
          yaw_1k = yaw_1 + 2 * M_PI;
        if (yaw_1 >= 0)
          yaw_1k = yaw_1;
        if (abs(yaw_2 - yaw_1) > abs(yaw_2 - yaw_1k))
        {
          // std::cout<<"turn angle is "<<(yaw_2-yaw_1k)<<"\n";
          turn((yaw_2 - yaw_1k) * 1);
        }
        else
        {
          // std::cout<<"turn angle is "<<(yaw_2-yaw_1)<<"\n";
          turn((yaw_2 - yaw_1) * 1);
        }
        i++;
        if (i >= goalSize)
          break;
      }

      // ori_dist=dist;
      while (ros::ok())
      {
        ros::Duration(0.1).sleep();
        ros::spinOnce();
        goal.target_pose.header.frame_id = "map";
        goal.target_pose.header.stamp = ros::Time::now();
        goal.target_pose.pose.position.x = goal_list[i * 4 + 0];    // goals[i][0];//goal_list[i*4+0]
        goal.target_pose.pose.position.y = goal_list[i * 4 + 1];    // goals[i][1];
        goal.target_pose.pose.orientation.z = goal_list[i * 4 + 2]; // goals[i][2];
        goal.target_pose.pose.orientation.w = goal_list[i * 4 + 3]; // goals[i][3];
        ac.sendGoal(goal);
        ROS_INFO("Goal %d set.", i);
        ac.waitForResult();

        if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        {
          ROS_INFO("Goal %d reached.", i);
        }
        else
        {
          ROS_INFO("Goal %d failed.", i);
        }
        break;

        loop_rate.sleep();
        time1 = ros::Time::now().toSec();
        std::cout << "pass time" << time1 - time0 << std::endl;
      }
    }
    ros::Duration(0.1).sleep();
    start.data = msg.data.back();
    start_node_pub.publish(start);
    msg.data.clear();
    goal_list.clear();
  }

  return EXIT_SUCCESS;
}

void turn(float goal_angle)
{
  double rate = 100;
  ros::Rate loopRate(rate);
  float angular_speed;
  angular_speed = copysign(0.3, goal_angle);
  float angular_duration = goal_angle / angular_speed;
  int count = 0;
  int ticks;
  geometry_msgs::Twist speed;

  while (ros::ok())
  {
    speed.linear.x = 0;
    speed.angular.z = angular_speed;
    ticks = int(angular_duration * rate);
    for (int i = 0; i < ticks; i++)
    {
      cmdVelPub.publish(speed);
      loopRate.sleep();
    }

    speed.angular.z = 0;
    cmdVelPub.publish(geometry_msgs::Twist());
    break;
  }
}
