#include<ros/ros.h>
#include<nav_msgs/Path.h>
#include <pbPlots.hpp>
#include <supportLib.hpp>
#include <tf/tf.h>
#include <iostream>
#include <cmath>
#include<vector>
#include <stdexcept>
#include <utility>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseStamped.h>

#define PI 3.141592653
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
using namespace std;
typedef std::pair<float, float> Point;
bool receive_one_msg_flag = 0;
vector<Point> pointList,pointListOut,orientation;


void path_callback(const nav_msgs::Path& msg){    
    float x,y;
    for(int i=0;i<msg.poses.size();i++){
        orientation.push_back(Point(msg.poses[i].pose.orientation.z,msg.poses[i].pose.orientation.w));
        // tf::Quaternion q(0,0,msg.poses[i].pose.orientation.z,msg.poses[i].pose.orientation.w);
        // angle.push_back(q.getAngle()*180/PI) ; 
        pointList.push_back(Point(msg.poses[i].pose.position.x,msg.poses[i].pose.position.y));
        cout<<i<<" ("<<pointList[i].first<<","<<pointList[i].second<<")"<<endl;       
    }
    receive_one_msg_flag=true;
}


// double PerpendicularDistance(const Point &pt, const Point &lineStart, const Point &lineEnd)
// {
// 	double dx = lineEnd.first - lineStart.first;
// 	double dy = lineEnd.second - lineStart.second;

// 	//Normalise
// 	double mag = pow(pow(dx,2.0)+pow(dy,2.0),0.5);
// 	if(mag > 0.0)
// 	{
// 		dx /= mag; dy /= mag;
// 	}

// 	double pvx = pt.first - lineStart.first;
// 	double pvy = pt.second - lineStart.second;

// 	//Get dot product (project pv onto normalized direction)
// 	double pvdot = dx * pvx + dy * pvy;

// 	//Scale line direction vector
// 	double dsx = pvdot * dx;
// 	double dsy = pvdot * dy;

// 	//Subtract this from pv
// 	double ax = pvx - dsx;
// 	double ay = pvy - dsy;

// 	return pow(pow(ax,2.0)+pow(ay,2.0),0.5);
// }

// void RamerDouglasPeucker(const vector<Point> &pointList, double epsilon, vector<Point> &out)
// {
// 	if(pointList.size()<2)
// 		throw invalid_argument("Not enough points to simplify");

// 	// Find the point with the maximum distance from line between start and end
// 	double dmax = 0.0;
// 	size_t index = 0;
// 	size_t end = pointList.size()-1;
// 	for(size_t i = 1; i < end; i++)
// 	{
// 		double d = PerpendicularDistance(pointList[i], pointList[0], pointList[end]);
// 		if (d > dmax)
// 		{
// 			index = i;
// 			dmax = d;
// 		}
// 	}

// 	// If max distance is greater than epsilon, recursively simplify
// 	if(dmax > epsilon)
// 	{
// 		// Recursive call
// 		vector<Point> recResults1;
// 		vector<Point> recResults2;
// 		vector<Point> firstLine(pointList.begin(), pointList.begin()+index+1);
// 		vector<Point> lastLine(pointList.begin()+index, pointList.end());
// 		RamerDouglasPeucker(firstLine, epsilon, recResults1);
// 		RamerDouglasPeucker(lastLine, epsilon, recResults2);
 
// 		// Build the result list
// 		out.assign(recResults1.begin(), recResults1.end()-1);
// 		out.insert(out.end(), recResults2.begin(), recResults2.end());
// 		if(out.size()<2)
// 			throw runtime_error("Problem assembling output");
// 	} 
// 	else 
// 	{
// 		//Just return start and end points
// 		out.clear();
// 		out.push_back(pointList[0]);
// 		out.push_back(pointList[end]);
// 	}
// }



int main(int argc, char**argv)
{
    ros::init(argc,argv,"tricycle_nav");
    ros::NodeHandle nh;
    ros::NodeHandle nh_priv("~");
    ros::Subscriber path = nh.subscribe("/move_base/TebLocalPlannerROS/global_plan",10,path_callback);
    MoveBaseClient ac("move_base", true);

    // ros::Subscriber path = nh.subscribe("/move_base/DWAPlannerROS/global_plan",10,path_callback);
    while(ros::ok()){
        if(receive_one_msg_flag==true)break;
        ros::spinOnce();       
    }

    // RamerDouglasPeucker(pointList, 0.1, pointListOut);
    

	// cout << "result" << endl;
    // for(size_t i=0;i< pointListOut.size();i++)
	// {      
	// 	cout<<pointListOut[i].first << "," << pointListOut[i].second << endl;

    // }

    return 0;
}