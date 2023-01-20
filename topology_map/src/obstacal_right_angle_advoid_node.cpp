#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Bool.h>
#include <std_msgs/UInt8.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <thread>
#include <math.h>

const float PI = 3.141592653589793238463;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
ros::Subscriber vel_sub, tag_sub, flag_sub ,local_plan_sub;
ros::Publisher flag_pub ,vis_goal_pub;
geometry_msgs::PoseStamped tagCamCoord, tagMapCoord ,vis_goal;
geometry_msgs::Twist cmd_vel;
move_base_msgs::MoveBaseGoal goal;
int tagID;
bool velCheck, tagCheck, goalCheck;
//param
std::string inputVelTopic, inputARTagTopic, inputFlagTopic, outputFlagTopic, inputplanTopic;
int targetTagID;
double targetDistance, publishFrequency;
std_msgs::Bool startFlag;
std_msgs::UInt8 finishFlag;

//for plancallback
double first_point[3];
double goal_point[2];
double right_angle_point[2];
float distance;
float a,b,c;//ax+by+c=0

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

double twopdis(float p1, float p2)//two point distence
{
	return sqrt(pow(p1,2)+pow(p2,2));
}

void velCallback(const geometry_msgs::Twist &vel_tmp)
{
	cmd_vel.linear.x = vel_tmp.linear.x;
	cmd_vel.linear.y = vel_tmp.linear.y;
	cmd_vel.angular.z = vel_tmp.angular.z;
}

void tagCallback(const visualization_msgs::Marker &tag_tmp)
{
	tagID = tag_tmp.id;
	tagCamCoord.header = tag_tmp.header;
	tagCamCoord.pose.position = tag_tmp.pose.position;
	tagCamCoord.pose.orientation = tag_tmp.pose.orientation;

}

void flagCallback(const std_msgs::Bool  &startFlag_tmp)
{
	startFlag.data = startFlag_tmp.data;
}

void planCallback(const nav_msgs::Path  &Path)
{
	int ranges = Path.poses.size();
	double firstyow, err = 180.0;
	firstyow = rad2deg(ori2rpy(Path.poses[0].pose));
	first_point[0] = Path.poses[0].pose.position.x;
	first_point[1] = Path.poses[0].pose.position.y;
	first_point[2] = firstyow;

	vis_goal.header = Path.header;
	vis_goal.pose.orientation = Path.poses[0].pose.orientation;

	/*
	for(int i = ranges*0.5; i < ranges; ++i)
	{
		double now_yaw = rad2deg(ori2rpy(Path.poses[i].pose));
		//ROS_INFO_STREAM(now_yaw);
		if(err > fabs(firstyow-now_yaw))
		{
			err =fabs(firstyow-now_yaw);
			goal_point[0] = Path.poses[i].pose.position.x;
			goal_point[1] = Path.poses[i].pose.position.y;
		}
	}
	*/
	goal_point[0] = Path.poses[ranges*0.5].pose.position.x;
	goal_point[1] = Path.poses[ranges*0.5].pose.position.y;

}


void planCallback_v2(const nav_msgs::Path  &Path)
{
	int ranges = Path.poses.size();
	double line_age, now_ang, err_eng;
	now_ang = rad2deg(ori2rpy(Path.poses[0].pose));
	line_age = rad2deg(atan((Path.poses[0].pose.position.y-Path.poses[1].pose.position.y)/
	                        (Path.poses[0].pose.position.x-Path.poses[1].pose.position.x)));
	err_eng = now_ang - line_age;
}
/*
void scanCallback(const sensor_msgs::LaserScan::ConstPtr &scan_tmp)
{
	scan.header.stamp = scan_tmp->header.stamp;
	scan.header.frame_id = scan_tmp->header.frame_id;
	scan.angle_min = scan_tmp->angle_min;
	scan.angle_max = scan_tmp->angle_max;
	scan.angle_increment = scan_tmp->angle_increment;
	scan.time_increment = scan_tmp->time_increment;
	scan.range_min = scan_tmp->range_min;
	scan.range_max = scan_tmp->range_max;
	int ranges = scan_tmp->ranges.size();
	scan.ranges.resize(ranges);
	for(int i = 0; i < ranges; ++i)
		scan.ranges[i] = scan_tmp->ranges[i];
}
*/

void goalSendThread()
{
	tf::TransformListener listener;
	MoveBaseClient ac("move_base", true);

    while(!ac.waitForServer(ros::Duration(5.0)))
        ROS_INFO("Waiting for the move_base action server to come up...");
    ROS_INFO("Connected to move base server.");

	while(ros::ok())
	{
		ros::spinOnce();
		
		if(tagID == targetTagID){
			tagCheck = 1;
			try{
				listener.transformPose("/map", tagCamCoord, tagMapCoord);
			}
			catch(tf::TransformException &ex){
				ros::Duration(0.1).sleep();
				tagCheck = 0;
				ROS_INFO("transformPose faild");
			}
			
		}


		velCheck = !(cmd_vel.linear.x + cmd_vel.angular.z);	//vel = 0, check = 1
		//if(velCheck && tagCheck && !goalCheck && startFlag.data)
		if(velCheck && tagCheck && startFlag.data)
		{
			tf::Quaternion q;
			double roll, pitch, yaw;
            tf::quaternionMsgToTF(tagMapCoord.pose.orientation, q);
            tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
			yaw +=1.5708;
			goal.target_pose.header.frame_id = "map";
			goal.target_pose.header.stamp = ros::Time::now();
            goal.target_pose.pose.position.x = tagMapCoord.pose.position.x - targetDistance*cos(yaw);
            goal.target_pose.pose.position.y = tagMapCoord.pose.position.y - targetDistance*sin(yaw);
            goal.target_pose.pose.position.z = 0;
			goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
			ac.sendGoal(goal);
			ac.waitForResult();
			if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
			{
				ros::spinOnce();
				ros::Duration(0.1).sleep();
				if(fabs(tagCamCoord.pose.orientation.z) < 0.1)
				{
					ROS_INFO("Goal reached and this node is alive.");
					goalCheck = 1;
					finishFlag.data = targetTagID;
					startFlag.data = false;
					flag_pub.publish(finishFlag);
				}
			}
			else
				ROS_INFO("Goal failed so sad.");
		}
		else
		{
			if(!velCheck)ROS_INFO("velCheck not yet.");
			if(!tagCheck)ROS_INFO("tagCheck not yet.");
			if(!startFlag.data)ROS_INFO("startFlag not yet.");
		}
		ros::Rate(publishFrequency).sleep();
	}
}


	
void shortest()
{
	//ax+by+c=0
	a = tan(deg2rad(first_point[2]));
	b = -1.0;
	c = first_point[1]-a*first_point[0];

	distance = ((a*goal_point[0]) + (b*goal_point[1]) + (c))/twopdis(a,b);//sqrt(pow(a,2)+1);
	float posible_a[2],posible_b[2];
	posible_a[0] = (cos(deg2rad(first_point[2]-90))*distance)+first_point[0];
	posible_a[1] = (sin(deg2rad(first_point[2]-90))*distance)+first_point[1];
	posible_b[0] = (cos(deg2rad(first_point[2]+90))*distance)+first_point[0];
	posible_b[1] = (sin(deg2rad(first_point[2]+90))*distance)+first_point[1];

	if(twopdis((posible_a[0]-goal_point[0]),(posible_a[1]-goal_point[1])) < twopdis( (posible_b[0]-goal_point[0]) , (posible_b[1]-goal_point[1]) ))
	{
		right_angle_point[0] = posible_a[0];
		right_angle_point[1] = posible_a[1];
	}
	else
	{
		right_angle_point[0] = posible_b[0];
		right_angle_point[1] = posible_b[1];
	}

	vis_goal.pose.position.x = right_angle_point[0];
	vis_goal.pose.position.y = right_angle_point[1];
	

}

void obs_dec_mode_timer(const ros::TimerEvent&)
{
	ROS_INFO_STREAM("first_point,"<<first_point[0]<<"  "<<first_point[1]<<"  "<<first_point[2]);
	ROS_INFO_STREAM("goal_point,"<<goal_point[0]<<"  "<<goal_point[1]);
	ROS_WARN_STREAM("distance = "<<distance);
	ROS_ERROR_STREAM("right_angle_point,"<<right_angle_point[0]<<"  "<<right_angle_point[1]);
	ROS_INFO_STREAM("--------------------------------------");
}

int main(int argc, char** argv){
	ros::init(argc, argv, "obstacal_right_angle_advoid_node");
	ros::NodeHandle nh;
	ros::NodeHandle nh_priv("~");
	ros::Rate mloop(30);
	nh_priv.param<std::string>("inputlocal_planTopic", inputplanTopic, "/move_base/TebLocalPlannerROS/local_plan");
	//nh_priv.param<std::string>("inputARTagTopic", inputARTagTopic, "/visualization_marker_updated");
	//nh_priv.param<std::string>("inputFlagTopic", inputFlagTopic, "/artag_navigation_start");
	//nh_priv.param<std::string>("outputFlagTopic", outputFlagTopic, "/artag_navigation_finished");
	//nh_priv.param<int>("targetTagID", targetTagID, 1);
	//nh_priv.param<double>("targetDistance", targetDistance, 0.5);
	//nh_priv.param<double>("publishFrequency", publishFrequency, 1.0);
	//vel_sub = nh.subscribe(inputVelTopic, 1000, &velCallback);
	//tag_sub = nh.subscribe(inputARTagTopic, 1, &tagCallback);
	//flag_sub = nh.subscribe(inputFlagTopic, 1, &flagCallback);
	local_plan_sub = nh.subscribe(inputplanTopic, 1, &planCallback);
	//flag_pub = nh.advertise<std_msgs::UInt8>(outputFlagTopic, 0);
	vis_goal_pub = nh.advertise<geometry_msgs::PoseStamped>("test_goal", 10);
	ROS_INFO("Artag navigation node opened.");
    //std::thread goalSend(goalSendThread);
	ros::Timer obsdec_mode_timer = nh.createTimer(ros::Duration(1.0), &obs_dec_mode_timer, false);
	while(ros::ok())
	{
		ros::spinOnce();
		shortest();


		vis_goal_pub.publish(vis_goal);
		mloop.sleep();
		
	}
	//goalSend.join();
	return EXIT_SUCCESS;
}
