#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseStamped.h>
#include"geometry_msgs/PoseWithCovarianceStamped.h"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <signal.h>
#include<math.h>
#include <cmath>
#include <vector>
ros::Publisher cmdVelPub;
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
geometry_msgs::PoseWithCovarianceStamped amcl;
void amclCallback(const geometry_msgs::PoseWithCovarianceStamped &Amcl){
  amcl=Amcl;
}
void turn(float goal_angle);
int main(int argc, char** argv)
{ 
// initialize ROS and the node
  ros::init(argc, argv, "multi_goal_loop_carrot");
  ros::NodeHandle nh;  
  ros::NodeHandle nh_priv("~");
  ros::Rate loop_rate(2);
  ros::Duration d(0.1);
  ros::Subscriber amcl_sub = nh.subscribe("amcl_pose", 2, amclCallback);//订阅
  float dist,ori_dist,temp_dist,carroat_dist=1.0,time0,time1;
  cmdVelPub=nh.advertise<geometry_msgs::Twist>("cmd_vel",1);
  MoveBaseClient ac("move_base", true);
  ROS_INFO("**********************************");

  // configuring parameters
  std::vector<float> goal_list;
  nh_priv.param("goal_list", goal_list, std::vector<float>());
  float turn_angle,yaw_1,yaw_2,w1,z1,w2,z2,yaw_1k;

  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  int goalSize = goal_list.size() / 4;
  ROS_INFO("goalSize=%d", goalSize);
  int goalFlag = 0;

  for(int i = 0; i < goalSize; i++)
    ROS_INFO("Goal list %d = [%.3f, %.3f, %.3f, %.3f]",i, goal_list[i*4+0], goal_list[i*4+1], goal_list[i*4+2], goal_list[i*4+3]);
  

  move_base_msgs::MoveBaseGoal goal; 
  for(int i=0; i<goalSize; i++)
  {
    ROS_INFO("navigation Goal list %d = [%.3f, %.3f, %.3f, %.3f]",i, goal_list[i*4+0], goal_list[i*4+1], goal_list[i*4+2], goal_list[i*4+3]);
    if(goal_list[i*4+0]==goal_list[(i-1)*4+0] && goal_list[i*4+1]==goal_list[(i-1)*4+1]){
      z1=goal_list[(i-1)*4+2];
      w1=goal_list[(i-1)*4+3];
      z2=goal_list[i*4+2];
      w2=goal_list[i*4+3];
      yaw_1=atan2(2*w1*z1,w1*w1-z1*z1);
      yaw_2=atan2(2*w2*z2,w2*w2-z2*z2);
      std::cout<<"yaw_1 = "<<yaw_1<<"\nyall_2 = "<<yaw_2<<"\n"; 
      if(yaw_1<0)yaw_1k=yaw_1+2*M_PI;
      if(yaw_1>=0)yaw_1k=yaw_1;
      if(abs(yaw_2-yaw_1)>abs(yaw_2-yaw_1k))
      {
        std::cout<<"turn angle is "<<(yaw_2-yaw_1k)<<"\n";
        turn(yaw_2-yaw_1k);
      }
      else
      {
      std::cout<<"turn angle is "<<(yaw_2-yaw_1)<<"\n";
      turn(yaw_2-yaw_1);
      }                 
      i++;
      if(i>=goalSize)break;
    }
      ros::spinOnce(); 
      dist=sqrt(pow(amcl.pose.pose.position.x-goal_list[i*4+0],2)+pow(amcl.pose.pose.position.y-goal_list[i*4+1],2));
      ori_dist=dist;   
    while(ros::ok()){
      //time0=ros::Time::now().toSec(); 
      dist=sqrt(pow(amcl.pose.pose.position.x-goal_list[i*4+0],2)+pow(amcl.pose.pose.position.y-goal_list[i*4+1],2));

      if(dist>carroat_dist){
        ros::spinOnce();
        dist=sqrt(pow(amcl.pose.pose.position.x-goal_list[i*4+0],2)+pow(amcl.pose.pose.position.y-goal_list[i*4+1],2));      
        ROS_INFO("current dist is %f",dist);
        goal.target_pose.header.frame_id = "map";
        goal.target_pose.header.stamp = ros::Time::now();
        goal.target_pose.pose.position.x =(amcl.pose.pose.position.x+(carroat_dist/dist)*(goal_list[i*4+0]-amcl.pose.pose.position.x));
        goal.target_pose.pose.position.y = (amcl.pose.pose.position.y+(carroat_dist/dist)*(goal_list[i*4+1]-amcl.pose.pose.position.y));
        goal.target_pose.pose.orientation.z = goal_list[i*4+2];
        goal.target_pose.pose.orientation.w = goal_list[i*4+3];
        
        std::cout<<"target goal x is "<<goal_list[i*4+0]<<"y is "<<goal_list[i*4+1]<<std::endl;
        std::cout<<"current amcl x is "<<amcl.pose.pose.position.x<<" y is "<<amcl.pose.pose.position.y<<std::endl;
        std::cout<<"goal x dt is "<<(carroat_dist/dist)*(goal_list[i*4+0]-amcl.pose.pose.position.x)<<"goal y dt is "<<(0.2/dist)*(goal_list[i*4+1]-amcl.pose.pose.position.y)<<std::endl;
        std::cout<<"temp goal x is "<<goal.target_pose.pose.position.x<<" y is "<<goal.target_pose.pose.position.y <<std::endl;
        
        ac.sendGoal(goal);
        while(ros::ok()){
          ros::spinOnce();
          temp_dist=sqrt(pow(goal.target_pose.pose.position.x-amcl.pose.pose.position.x,2)+pow(goal.target_pose.pose.position.y-amcl.pose.pose.position.y,2));
          std::cout<<"temp_dist is "<<temp_dist;
          if(temp_dist<(carroat_dist/2.0))
            break;         
          else if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            break;
        }

        
      }
      else{
        goal.target_pose.header.frame_id = "map";
        goal.target_pose.header.stamp = ros::Time::now();
        goal.target_pose.pose.position.x = goal_list[i*4+0];//goals[i][0];//goal_list[i*4+0]
        goal.target_pose.pose.position.y = goal_list[i*4+1];//goals[i][1];
        goal.target_pose.pose.orientation.z = goal_list[i*4+2];//goals[i][2];
        goal.target_pose.pose.orientation.w = goal_list[i*4+3];//goals[i][3];
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
        break;
      }
      loop_rate.sleep();
      time1=ros::Time::now().toSec();
      std::cout<<"pass time"<<time1-time0<<std::endl;
    }


  }


  return EXIT_SUCCESS;
}

void turn(float goal_angle){
    double rate = 100;
    ros::Rate loopRate(rate);
    float angular_speed;
    angular_speed=copysign(0.3,goal_angle);
    /*if(goal_angle>0)
      angular_speed=0.3;
    else
      angular_speed=-0.3;
    */
    float angular_duration = goal_angle / angular_speed;
    int count =0;
    int ticks;
    geometry_msgs::Twist speed;

    while(ros::ok())
    {
      speed.linear.x=0;
      speed.angular.z=angular_speed;
      ticks = int(angular_duration*rate);
      for(int i=0;i < ticks;i++)
      {
          cmdVelPub.publish(speed);
          loopRate.sleep();
      }
       
      speed.angular.z = 0;
      cmdVelPub.publish(geometry_msgs::Twist());
      break;
    }
}