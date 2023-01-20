#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include <math.h>
#include <geometry_msgs/Twist.h>
#include "nav_msgs/Odometry.h"
using namespace std;
float yaw_1, yaw_2;
bool goal1_flag = false, goal2_flag = false;
bool triggeer = false;
float pre_odom, odom_pass;
float yaw_r;

void goal1Callback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
  goal1_flag = true;
  ROS_INFO("goal1 %f %f %f %f\n", msg->pose.position.x, msg->pose.position.y, msg->pose.orientation.z, msg->pose.orientation.w);
  float z1 = msg->pose.orientation.z;
  float w1 = msg->pose.orientation.w;
  yaw_1 = atan2(2 * w1 * z1, w1 * w1 - z1 * z1);
  // yaw_1 = format_angle(yaw_1);
  if (yaw_1 < 0)
    yaw_1 = yaw_1 + 2 * M_PI;
  std::cout << "yaw_1 = " << yaw_1 * 180 / M_PI << "\n";
}

void goal2Callback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
  goal2_flag = true;
  ROS_INFO("goal2 %f %f %f %f\n", msg->pose.position.x, msg->pose.position.y, msg->pose.orientation.z, msg->pose.orientation.w);
  float z2 = msg->pose.orientation.z;
  float w2 = msg->pose.orientation.w;
  yaw_2 = atan2(2 * w2 * z2, w2 * w2 - z2 * z2);
  if (yaw_2 < 0)
    yaw_2 = yaw_2 + 2 * M_PI;
  std::cout << "\nyall_2 = " << yaw_2 * 180 / M_PI << "\n";
}

void odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
  float rz, rw;
  rz = msg->pose.pose.orientation.z;
  rw = msg->pose.pose.orientation.w;
  yaw_r = atan2(2 * rw * rz, rw * rw - rz * rz);
  if (yaw_r < 0)
    yaw_r = yaw_r + 2 * M_PI;
  if (triggeer == false)
  {
    pre_odom = yaw_r;
    triggeer = true;
  }
  if (abs(yaw_r - pre_odom) > 1)
    odom_pass = odom_pass;
  else
    odom_pass += yaw_r - pre_odom;
  pre_odom = yaw_r;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "spin_node");
  ros::NodeHandle nh;

  // Subscribe to move_base_simple/goal1 and move_base_simple/goal2 topics
  ros::Subscriber goal1_sub = nh.subscribe("move_base_simple/goal1", 1000, goal1Callback);
  ros::Subscriber goal2_sub = nh.subscribe("move_base_simple/goal2", 1000, goal2Callback);
  ros::Subscriber odom_sub = nh.subscribe("/odom", 1000, odomCallback);
  ros::Publisher cmdVelPub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);

  while (goal1_flag == false || goal2_flag == false)
  {
    ros::Duration(0.1).sleep();
    ros::spinOnce();
  }

  float desire_angle;
  desire_angle = yaw_2 - yaw_1;
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
    ros::Duration(0.1).sleep();
    ros::spinOnce();
  }
  cout << "odom_pass" << odom_pass * 180 / M_PI << endl;
  cmdVelPub.publish(geometry_msgs::Twist());
  triggeer = false;

  return 0;
}
