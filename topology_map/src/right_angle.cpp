#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_datatypes.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv)
{
  // initialize ROS and the node
  ros::init(argc, argv, "multi_goal_loop");
  ros::NodeHandle nh;
  ros::NodeHandle nh_priv("~");
  ros::Rate r(30);
  ros::Duration d(0.1);
  MoveBaseClient ac("move_base", true);

  // configuring parameters
  std::vector<float> goal_list;
  nh_priv.param("goal_list", goal_list, std::vector<float>());
  float goals[][4] = {};
  int j, k, size = 0;
  for(int i = 0; i < goal_list.size(); i++)
  {
    j = i/4;
    k = i%4;
    goals[j][k]=goal_list[i];
    size++;
  }

  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  int goalSize = goal_list.size() / 4;
  //int goalSize = size / 4;
  ROS_INFO("goalSize=%d", goalSize);
  int goalFlag = 0;

  ROS_INFO("Goal 0 = [%.3f, %.3f, %.3f, %.3f]", goals[0][0], goals[0][1], goals[0][2], goals[0][3]);

  move_base_msgs::MoveBaseGoal goal;
  while(ros::ok())
  {
    for(int i=0; i<goalSize; i++)
    {
      goal.target_pose.header.frame_id = "map";
      goal.target_pose.header.stamp = ros::Time::now();
      goal.target_pose.pose.position.x = goals[i][0];
      goal.target_pose.pose.position.y = goals[i][1];
      goal.target_pose.pose.orientation.z = goals[i][2];
      goal.target_pose.pose.orientation.w = goals[i][3];
      //goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
      ac.sendGoal(goal);
      ROS_INFO("Goal %d set.", i);
      ac.waitForResult();
      if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
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
  }

  return EXIT_SUCCESS;
}