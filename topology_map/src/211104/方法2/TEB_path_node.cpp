#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Bool.h>
#include <std_msgs/UInt8.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <thread>
#include <math.h>

const float PI = 3.141592653589793238463;

ros::Subscriber local_plan_sub;
ros::Publisher vis_goal_pub;
geometry_msgs::PoseStamped vis_goal;
//param
std::string inputplanTopic;
int path_size;
int path_point;


double deg2rad(double angle_in_degrees)
{
	return angle_in_degrees*PI/180;
}

double rad2deg(double angle_in_radians)
{
	return angle_in_radians*180/PI;
}

double ori2rpy(const geometry_msgs::Pose in_p)
{
	tf::Quaternion q;
	double roll, pitch, yaw;
	tf::quaternionMsgToTF(in_p.orientation, q);
	tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
	return yaw;
}






void planCallback(const nav_msgs::Path  &Path)
{
	path_size = Path.poses.size();
	path_point > path_size ? path_size : path_point 
	//0 : robot position 
	vis_goal.header = Path.header;
	vis_goal.pose.orientation = Path.poses[path_point >= path_size ? path_size-1 : path_point ].pose.orientation;
	vis_goal.pose.position = Path.poses[path_point >= path_size ? path_size-1 : path_point ].pose.position;

}




void obs_dec_mode_timer(const ros::TimerEvent&)
{
	ROS_INFO("path_size %d",path_size);
	ROS_INFO("path_point %d",path_point);
	ROS_INFO_STREAM("--------------------------------------");
}

int main(int argc, char** argv){
	ros::init(argc, argv, "get_path_node");
	ros::NodeHandle nh;
	ros::NodeHandle nh_priv("~");
	ros::Rate mloop(30);
	nh_priv.param<std::string>("inputlocal_planTopic", inputplanTopic, "/move_base/TebLocalPlannerROS/local_plan");
	nh_priv.param<int>("path_point", path_point, 0);

	local_plan_sub = nh.subscribe(inputplanTopic, 1, &planCallback);
	vis_goal_pub = nh.advertise<geometry_msgs::PoseStamped>("TEB_pose", 10);
	ROS_INFO("Getting TEB path node opened.");
	ROS_INFO("path_point %d",path_point);
	
	//ros::Timer obsdec_mode_timer = nh.createTimer(ros::Duration(1.0), &obs_dec_mode_timer, false);
	while(ros::ok())
	{
		ros::spinOnce();


            vis_goal_pub.publish(vis_goal);

        mloop.sleep();
		
	}
	//goalSend.join();
	return EXIT_SUCCESS;
}
