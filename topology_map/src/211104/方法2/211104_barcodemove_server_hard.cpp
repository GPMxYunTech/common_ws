// last edit 20210119
/***Alarm code definition
 * 0:NoAlarm
 * 1:WrongReceivedMsg
 * 2:WrongExtendPath
 * 3:OutOfLineWhileForwardingEnd
 * 4:OutOfLineWhileTrackingEndPoint
 * 5:OutOfLineWhileMoving
 * 6:OutOfLineWhileSecondary
 * 7:MissingTagOnEndPoint
 * 8:MissingTagWhileMoving
 * 9:MissingTagWhileSecondary
 * 10:WrongInitialPositionInSecondary
 * 11:WrongInitialAngleInSecondary
 * 90:The normal navigation path cannot continue,UI chiose whether avoid path
 * 91:The obstacle avoidance path cannot continue,UI decide back to previous point
 * 92:Still cannot move,need operator get involved
***/

#include "ros/ros.h"
#include <vector>
#include <string>
#include <std_msgs/Char.h>
#include <sensor_msgs/LaserScan.h> //20210330GU
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int16.h> /* 210511GU */
#include <std_msgs/UInt8.h> //20210205 char
#include <std_msgs/Bool.h>  //20210205 char
#include <actionlib/server/simple_action_server.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_msgs/GetPlan.h>
#include "nav_msgs/Path.h"
#include <nav_core/base_local_planner.h>
#include "gpm_msgs/ComplexRobotControlCmd.h"
#include "gpm_msgs/NavigationState.h"
#include "gpm_msgs/BarcodeReaderState.h"
#include "gpm_msgs/TaskCommandAction.h"
#include "gpm_msgs/PathInfo.h"
#include "gpm_msgs/SetcurrentTagID.h"     /* 210331jimc */
#include "gpm_msgs/PoseStamped.h"         /* 210409jimc */
#include <glog/logging.h>
#include <move_base_msgs/MoveBaseActionGoal.h>
#include <move_base_msgs/MoveBaseActionResult.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <visualization_msgs/Marker.h>    //artag
#include <artag_msgs/artag_path.h>        //artag
#include <artag_msgs/stat.h>              //artag
#include "std_srvs/Empty.h"

#define Sign(A) ((A) >= 0 ? 1 : -1)


void change_nav_param(int);   /* 210407jimc */

double looprate = 30; 
double velrate;    // 0.5~1 , for service use to down low speed (vel_max)
double speedmax;   // the default maximun work velocity
double dis_th;     // for line distance threshold < 3cm as on line
double ang_th;     // for angle threshold < (2 deg) in rad as on angle
double linedis_th; // for line distance threshold in long line moving, over it will tracking error
double lineang_th; // for angle threshold < (12 deg) long line moving, over it will tracking error
double vel_low;    // min velocity in m/s

double vel_max;    // max work velocity , verlrate as service call
double vel_Secondary;
double vel_SecondaryMax;
double vel_SecondarySlow;
double vel_Secondary_minimum_angular_speed;
double velacel_secondary;
double wan_low;                    // min angular velocity as (2 deg/s) in rad/s
double wan_max;                    // max angular velocity
double wanacel;                    // rad/ss, every loop add wanacel/looprate )

double velacel;                    // m/ss, every loop add velacel/looprate )
double veldecel;                   // engency stop accel (1.5 m /ss)
double veldecel_emergency;         // forward stop accel (1.0  m /ss)
double slowfinaldist;              // final keep vel_low distance for last target
double slowdownang;                // angular velocity slow down angle threshold
double lineacc_th;                 // line velocity acceleration
double angacc_th;                  // angular velocity acceleration
double line_kp;                    // angluar p control in line (robtolinang_err-robtrackangerr)*line_kp
double dline_kp;                   // angular p control in mode3/4  (docking)
double pangle;                     // 1.5 deg per 1cm for line track
double offset_last_angle_adjusted; // turning to the setting offset angle
double offset_line_tracking;       // offset of tracking line in normal type
double offset_tape_tracking_BH;    // offset of tracking tape in backend homing trajectory
//
bool gettag = false;    // get bcr tag flag
int tagcount = 0;       // get bcr tag counter for debounce
bool gettap = false;    // get bcr tape flag
double tapydis = 0;     // get bcr y distance in tape
float tapang = 0;       // get bcr angle in tap
double tagxdis = 0;     // get bcr x distance in tag
double tagydis = 0;     // get bcr y distance in tag
float tagang = 0;       // get bcr angle in tag
int TagID = 0;          // get bcr Tag ID
int BcrRetryCount;      // get tag turn into false when bcr continuously read fail in BcrRetryCount/bcr_loop_rate sec
uint16_t robdir = 0;    // flag for dir lamp, forward=0, ccw RoW>0 =1, cw RoW<0 =2
uint16_t allowpath = 0; // msg for allow path send (1) ok, (0) don't send
double last_first_x;
double last_first_y;
uint16_t lastfinalgoalid; //dynpath, keep last global final id
uint16_t AlarmCode = 0;
int lastVisitedNode = 0;
float map_score_limit;
float map_score_high;
float map_score_low;
float map_score = 0;      /* 210513jimc */
bool corner_flag = 0;     //0427 char 

int abortcount = 0;             /* 210208jimc */
int artagState = 0;             /* 0:Normal, 1:Moving, 2:Finish, 3:Error */ /* 210208jimc */
int nav_param_select=-1;           /* 210205jimc */
int nav_param_select_previou=-1;   /* 210205jimc */

int artagexe = 0;               /* 210423jimc */
bool switvh_bay_flag =false;
/*  obstacle_deceleration  */
int obstacle_deceleration_mode = 0;           /* 210331jimc */
int pre_obstacle_deceleration_mode = -1;      /* 210331jimc */
int misstion_exe_flg = 0;                     /* 210401jimc */
int obsdec_avoid_flg = 0;                     /* 210401jimc */
int pre_obsdec_avoid_flg = 0;                 /* 210401jimc */
int obsdecState = 0;                          /* 210413jimc */
int switch_flag=0;
int instrumentState = 0;                      /* 210428jimc */
int velctrl_flag;              /* 210927char */
double velctrl_flag_first_vel;
int velctrl_flag_count = 0;    /* 210928char */
float slowdown_vel = 0.2;                     /* 211013jimc */

/* switch map */
std::string switch_map_name;                  /* 210409jimc */
std::string pre_switch_map_name;              /* 210409jimc */

tf::Stamped<tf::Pose> obsdec_grobot_pose;     /* 210401jimc */
tf::Stamped<tf::Pose> obsdec_lastgoal_pose;   /* 210401jimc */

gpm_msgs::PoseStamped switch_map_inf;         /* 210409jimc */


costmap_2d::Costmap2DROS *planner_costmap_ros_, *controller_costmap_ros_;
typedef actionlib::SimpleActionServer<gpm_msgs::TaskCommandAction> Server;
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

/*-------------*/
/*  Publisher  */
/*-------------*/
ros::Publisher plan_pub;
ros::Publisher goal_pub;
ros::Publisher velocity_pub;
ros::Publisher artag_move_flag_pub;               /* 210205jimc */
ros::Publisher artag_tagID_pub;                   /* 210205jimc */
ros::Publisher artag_func_flag_pub;               /* 210205jimc */
ros::Publisher obstacle_deceleration_mode_pub;    /* 210331jimc */
ros::Publisher map_update_pub;                    /* 210331 char */
ros::Publisher map_switch_pub;                    /* 210421 char */
ros::Publisher bay_alignment_pub;                 /* 210511GU */
ros::Publisher obst_mode_pub;                 /* 210927 char */

/*-------------*/
/*  Subscribe  */
/*-------------*/
ros::Subscriber goal_result_sub;
ros::Subscriber slam_cmd_vel_sub;
ros::Subscriber laser_sub;                        //20210330GU
ros::Subscriber bay_response_sub;                 /* 210511GU */
ros::Subscriber artag_stat_sub;                   /* 210205jimc */
ros::Subscriber vel_sub;                          /* 210324jimc */
ros::Subscriber map_score_sub;                    //0331
ros::Subscriber obstacle_deceleration_status_sub; /* 210413jimc */
ros::Subscriber robot_pose_now_sub;               //0427 char 
ros::Subscriber instrument_status_sub;            /* 210428jimc */
ros::Subscriber velctrl_sub;            /* 210927 char */
ros::Subscriber TEB_path_sub ;  //1104 char  /* teb_path */
/*------------------*/
/*  Service client  */
/*------------------*/
ros::ServiceClient switch_map_client;             /* 210409jimc */

//pause-forward stop flag
bool pausestop = 0;
bool forwardstop = 0;
bool forwardstate = 0;
bool remathstop = 0; //0331
//dynpath, flag, line segment, subitem id
bool dynflag = 0;
bool lastdynflag = 0;
int lastsegid = 0;
int lastsubid = 0;
//check and receive tag map path from bcr
bool isSlowDown = false;
bool isRoughAdjust = false;
bool isErrorSlowDown = false;
bool isFindingTag = false;

//0115 tag function flag
bool search_tag;
bool tracking_tag_center;

//move_base result 
bool goal_result;
bool get_slam_cmd = 0;

move_base_msgs::MoveBaseGoal replan_goal; //20210303 char
int replan_goal_flag = 0;                 //20210303 char
int back_flag=0;
int obst_back=0;
geometry_msgs::Twist current_cmd_vel;
geometry_msgs::Twist fixed_cmd_vel;       //20210111wusl
geometry_msgs::Twist slam_cmd_vel;
//20210111wusl add start
// configuring parameters
double goalRange1=0.0, goalRange2=0.0, goalRange3=0.0, goalRange4=0.0, goalRange5=0.0;
double goalSpeed1=1.0, goalSpeed2=1.0, goalSpeed3=1.0, goalSpeed4=1.0, goalSpeed5=1.0;
  
int map_score_cunt = 0;     /*0331 char check map score*/
int map_score_low_count = 0; /* 210913jimc */
double laser_left = 0.0;     //20210330GU
double laser_right = 0.0;    //20210330GU
bool bay_flag = 0;           /* 210511GU */
bool pass_point_check = false;  /* 210907jimc */

geometry_msgs::PoseStamped robot_pose;  /* 210421 char */
geometry_msgs::Pose TEB_path_pose; //1104 char  /* teb_path */
/*
  nh_priv.param<double>("goalRange1", goalRange1, 0.0);
  nh_priv.param<double>("goalRange2", goalRange2, 0.0);
  nh_priv.param<double>("goalRange3", goalRange3, 0.0);
  nh_priv.param<double>("goalRange4", goalRange4, 0.0);
  nh_priv.param<double>("goalRange5", goalRange5, 0.0);
  nh_priv.param<double>("goalSpeed1", goalSpeed1, 1.0);
  nh_priv.param<double>("goalSpeed2", goalSpeed2, 1.0);
  nh_priv.param<double>("goalSpeed3", goalSpeed3, 1.0);
  nh_priv.param<double>("goalSpeed4", goalSpeed4, 1.0);
  nh_priv.param<double>("goalSpeed5", goalSpeed5, 1.0);
*/
//20210111wusl add end


template <typename T>
T getParam(const std::string &name, const T &defaultValue) //This name must be namespace+parameter_name
{
  T v;
  if (ros::param::get(name, v)) //get parameter by name depend on ROS.
  {
    LOG(INFO) << "Found parameter: Key=" << name << ", Value=" << v;
    return v;
  }
  else
    LOG(INFO) << "Not find value for parameter: Key=" << name << ", Default Value=" << defaultValue;
  return defaultValue; //if the parameter haven't been set,it's value will return defaultValue.
}

void ReadParam()
{
  std::string agvname = getParam<std::string>("agvname", "agv000");
  std::string retrievingname = "/barcodemove_server";

  LOG(INFO) << "========Reading Movebase Parameter========";
  velrate = getParam<double>(retrievingname + "/velrate", 0.1);
  speedmax = getParam<double>(retrievingname + "/speedmax", 0.08);
  dis_th = getParam<double>(retrievingname + "/dis_th", 0.03);
  ang_th = getParam<double>(retrievingname + "/ang_th", 1 * M_PI / 180);
  linedis_th = getParam<double>(retrievingname + "/linedis_th", 0.1);
  lineang_th = getParam<double>(retrievingname + "/lineang_th", 12 * (M_PI / 180));
  vel_low = getParam<double>(retrievingname + "/vel_low", 0.04);

  vel_max = speedmax;
  vel_SecondaryMax = getParam<double>(retrievingname + "/vel_SecondaryMax", 0.3);
  vel_SecondarySlow = getParam<double>(retrievingname + "/vel_SecondarySlow", 0.035);
  vel_Secondary_minimum_angular_speed = getParam<double>(retrievingname + "/vel_SecondaryMinAngularSpeed", 0.003);
  velacel_secondary = getParam<double>(retrievingname + "/velacel_Secondary", 0.5);
  vel_Secondary = vel_SecondaryMax;
  wan_low = getParam<double>(retrievingname + "/wan_low", 4.8 * M_PI / 180);
  wan_max = getParam<double>(retrievingname + "/wan_max", 15 * M_PI / 180);
  wanacel = getParam<double>(retrievingname + "/wanacel", 3 * M_PI / 180);


  velacel = getParam<double>(retrievingname + "/velacel", 0.15);
  veldecel = getParam<double>(retrievingname + "/veldecel", 1.5);
  veldecel_emergency = getParam<double>(retrievingname + "/veldecel_emergency", 1.0);
  slowfinaldist = getParam<double>(retrievingname + "/slowfinaldist", 0.2);
  slowdownang = getParam<double>(retrievingname + "/slowdownang", 30 * M_PI / 180);
  lineacc_th = getParam<double>(retrievingname + "/lineacc_th", 0.05);
  angacc_th = getParam<double>(retrievingname + "/angacc_th", 5 * M_PI / 180);
  line_kp = getParam<double>(retrievingname + "/line_kp", 0.7);
  dline_kp = getParam<double>(retrievingname + "/dline_kp", 1.0);
  pangle = getParam<double>(retrievingname + "/pangle", (2.0 * M_PI / 180) / 0.01);
  BcrRetryCount = getParam<double>(retrievingname + "/BcrRetryCount", 40);
  offset_last_angle_adjusted = getParam<double>(retrievingname + "/LastPointAngleOffset", 0);
  offset_line_tracking = getParam<double>(retrievingname + "/offset_LineTracking", 0);
  offset_tape_tracking_BH = getParam<double>(retrievingname + "/offset_TapeTrackingInBH", 0);

  search_tag = getParam<double>(retrievingname + "/search_tag", 0);
  tracking_tag_center = getParam<double>(retrievingname + "/tracking_tag_center", 1);
  LOG(INFO) << "======End Reading Movebase Parameter======";
}
//error calculate function
double diserr(const tf::Stamped<tf::Pose> &targetpose, const tf::Stamped<tf::Pose> &robotpose)
{
  /* 210223jimc return sqrt((targetpose.getOrigin().x() - robotpose.getOrigin().x()) * (targetpose.getOrigin().x() - robotpose.getOrigin().x()) + 		      (targetpose.getOrigin().y() - robotpose.getOrigin().y()) * (targetpose.getOrigin().y() - robotpose.getOrigin().y())); */
  return sqrt( pow( (targetpose.getOrigin().x() - robotpose.getOrigin().x()), 2 ) + 
               pow( (targetpose.getOrigin().y() - robotpose.getOrigin().y()), 2 )   );  /* 210223jimc */
}

float CalculateLineAngle(const tf::Stamped<tf::Pose> &p0, const tf::Stamped<tf::Pose> &p1)
{
  //printf("p1 x %f ,p1 y %f,p0 x %f,p0 y %f",p1.getOrigin().x(),p1.getOrigin().y(),p0.getOrigin().x(),p0.getOrigin().y());
  return atan2((p1.getOrigin().y() - p0.getOrigin().y()), (p1.getOrigin().x() - p0.getOrigin().x()));
}

float Calculate2LineAngleDiff(const tf::Stamped<tf::Pose> &p0, const tf::Stamped<tf::Pose> &p1, const tf::Stamped<tf::Pose> &p2)
{
  float line1ang, line2ang, angleerr;
  line1ang = CalculateLineAngle(p1, p0);
  line2ang = CalculateLineAngle(p1, p2);
  angleerr = line2ang - line1ang;
  angleerr = atan2(sin(angleerr), cos(angleerr));
  return angleerr;
}

float angerr(const tf::Stamped<tf::Pose> &targetpose, const tf::Stamped<tf::Pose> &robotpose)
{
  float robotang = tf::getYaw(robotpose.getRotation());	 	  //-pi ~ pi
  float goalang = CalculateLineAngle(robotpose, targetpose); 	//-pi ~ pi    //delt ang 20210308
  //float goalang = tf::getYaw(targetpose.getRotation()); 	//-pi ~ pi    //target ang  20210204 char

  float angleerr = goalang - robotang;
  return atan2(sin(angleerr), cos(angleerr));
}

float frangerr(const tf::Stamped<tf::Pose> &targetpose, const tf::Stamped<tf::Pose> &robotpose)
{
  float angleerr = tf::getYaw(targetpose.getRotation()) - tf::getYaw(robotpose.getRotation());
  return atan2(sin(angleerr), cos(angleerr));
}

double robtolindiserr(const tf::Stamped<tf::Pose> &p0, const tf::Stamped<tf::Pose> &p1, const tf::Stamped<tf::Pose> &pr)
{
  double vax = p1.getOrigin().x() - p0.getOrigin().x();
  double vay = p1.getOrigin().y() - p0.getOrigin().y();
  double vbx = pr.getOrigin().x() - p0.getOrigin().x();
  double vby = pr.getOrigin().y() - p0.getOrigin().y();
  double valength = sqrt(vax * vax + vay * vay);
  if (valength == 0)
    return sqrt(vbx * vbx + vby * vby);
  return (vax * vby - vay * vbx) / valength; // dis >0 in left of line, dis=0 in line , dis < 0 in right
}

float robtolinangerr(const tf::Stamped<tf::Pose> &p0, const tf::Stamped<tf::Pose> &p1, const tf::Stamped<tf::Pose> &pr, const float &offset = 0)
{
  if (p1.getOrigin().x() == p0.getOrigin().x() && p1.getOrigin().y() == p0.getOrigin().y())
    return 0.0;
  float robotang = tf::getYaw(pr.getRotation()) - offset;
  float linang = CalculateLineAngle(p0, p1);
  float angleerr = linang - robotang;
  return atan2(sin(angleerr), cos(angleerr));
}

double AdjustRunningRoX(double RoX, double robtolindis_err,double robtolinang_err, bool isGoback = false)
{
  if(isGoback)robtolindis_err *=-1;
  double kp = 0.5;//Vy_adj_kp;
  RoX = (robtolindis_err) * kp;
  if (fabs(RoX) >= 0.05 )RoX = Sign(RoX) * 0.05 ;

  printf("AdjustRunningRoX=%f robtolindis_err=%f\n",RoX,robtolindis_err);
  return RoX;
}

double AdjustRunningRoW(double RoW, double robtolindis_err, double robtolinang_err, bool isSecondary = false, bool isGoBack = false)
{
  double robtrackangerr = 0;
  double line_deviation = isSecondary ? 0.002 : 0.02; //Unit: m
  double kp = isSecondary ? dline_kp : line_kp;

  if (fabs(robtolindis_err) <= line_deviation)
    robtrackangerr = 0;
  else
    robtrackangerr = Sign(robtolindis_err) * fabs(fabs(robtolindis_err)) * pangle; // pangle

  if (isGoBack)
  {
    if (fabs(robtolinang_err) >= 179.8 * M_PI / 180)
      robtolinang_err = 0;
    else
    {
      robtolinang_err = M_PI - robtolinang_err;
      if (robtolinang_err > M_PI)
        robtolinang_err = robtolinang_err - 2 * M_PI;
    }
    robtolinang_err *= -1;
  }

  RoW = (robtolinang_err - robtrackangerr) * kp; //***0

  if (fabs(RoW) >= wan_max / 4)
    RoW = Sign(RoW) * wan_max / 4;

  return RoW;
}

void SetAGVDirection(const double &ang_err)
{
  /* 210324jimc mark
  if (ang_err == 0)
    robdir = 0;
  else if (ang_err > 0)
    robdir = 1;
  else
    robdir = 2;
    210324jimc mark end */
    }

bool Turning_loop(double &ang_vel, const float &ang_err)
{
  
    //RoW up
  SetAGVDirection(ang_err);
  if (fabs(ang_err) > slowdownang)
    ang_vel = Sign(ang_err) * wan_max;
  //RoW down
  else
  {
    
    ang_vel = Sign(ang_err) * wan_low;
    if (fabs(ang_err) <= ang_th)
    {
      ang_vel = 0;
      SetAGVDirection(0);
      LOG(INFO) << "Turning done";
      return true;
    }
  }
  //printf("turning %f\n",ang_vel); //0209 test
  return false;
}
//0317
bool fix_disYerr(double &y_vel,const float &disY_err)
{
  
  
  //move y
  //SetAGVDirection(ang_err);
  if (fabs(disY_err) > slowdownang)
    y_vel = disY_err;
  //RoW down
  else
  {
    
    y_vel =disY_err;
    if (fabs(disY_err) <= dis_th)
    {
      y_vel = 0;
      //SetAGVDirection(0);
      LOG(INFO) << "Moving done";
      return true;
    }
  }
  printf("Y move %f\n",y_vel); 
  return false;
}
//0317 end
//0315
/* 210824jimc bool fix_diserr(double &x_vel,double &y_vel, const float &disX_err,const float &disY_err)*/
bool fix_diserr(double &x_vel,double &y_vel, const double &disX_err,const double &disY_err) /* 210824jimc */
{
  
  //move x 
  //SetAGVDirection(ang_err);
  if (fabs(disX_err) > slowfinaldist)
    x_vel = Sign(disX_err) * vel_max;
  //RoW down
  else
  {
    printf("x move %f\n",disX_err); 
    x_vel = Sign(disX_err) * vel_low;
    if (fabs(disX_err) <= dis_th)
    {
      x_vel = 0;
      //SetAGVDirection(0);
      LOG(INFO) << "Moving done";
      return true;
    }
  }

  //move y
  //SetAGVDirection(ang_err);
  if (fabs(disY_err) > slowdownang)
    y_vel = Sign(disY_err) * vel_max;
  //RoW down
  else
  {
    printf("Y move %f\n",disY_err); 
    y_vel = Sign(disY_err) * vel_low;
    if (fabs(disY_err) <= dis_th)
    {
      y_vel = 0;
      //SetAGVDirection(0);
      LOG(INFO) << "Moving done";
      return true;
    }
  }
  return false;
}
//0316 end

bool IsTrackingOnLine(const double &robtolindis_err, const double &robtolinang_err, bool isGoBack = false)
{
  if (isGoBack)//isGoBack && gettap
    return fabs(robtolindis_err) <= linedis_th && fabs(robtolinang_err) >= (M_PI - lineang_th);
  //else if (isGoBack)
    //return fabs(robtolindis_err) <= linedis_th && fabs(robtolinang_err) <= lineang_th;
  else
    return fabs(robtolindis_err) <= linedis_th && fabs(robtolinang_err) <= lineang_th;
}

bool IsErrorStopDone(geometry_msgs::Vector3 &_linear_vel, double &_row, const geometry_msgs::Twist &vel)
{
  _linear_vel.x = _linear_vel.y = _row = 0;
  return vel.linear.x == 0 && vel.linear.y == 0 && vel.angular.z == 0;
}

bool IsAGVOnLineDirection(const nav_msgs::Path &planpath)
{
  if (planpath.poses.size() < 2)
    return false;
  tf::Stamped<tf::Pose> p0, p1, agvpose;
  planner_costmap_ros_->getRobotPose(agvpose);
  poseStampedMsgToTF(planpath.poses[0], p0);
  poseStampedMsgToTF(planpath.poses[1], p1);
  //because "robtolinangerr" return 0 when p0 is the same to p1, add this function to avoid wrong case
  if (p1.getOrigin().x() == p0.getOrigin().x() && p1.getOrigin().y() == p0.getOrigin().y())
    return false;
  float robtoline_err = robtolinangerr(p0, p1, agvpose);
  //printf("robtoline_err=%f\n",robtoline_err);
  LOG(INFO) << "[IsAGVOnLineDirection] Check the line angle err : " << robtoline_err;
  return fabs(robtoline_err) < 20 * M_PI / 180;
}

bool IsOverTarget(const bool &mode, const float &ang_err)
{
  if (mode)
    return fabs(ang_err) < 1.48;
  return fabs(ang_err) > 1.48;
}

bool IsOverTarget(const double &next_dist, const float &next_ang)
{
  return next_dist > slowfinaldist && fabs(next_ang) > M_PI / 2;
}

bool IsCompletedTask(const uint16_t &currentFinalID, const uint16_t &finalID)
{
  if (currentFinalID == finalID)
  {
    LOG(INFO) << "get a completed task";
    return 0;
  }
  LOG(INFO) << "get a uncompleted task";
  return 1;
}

bool IsCorrectExtentPath(const geometry_msgs::Point &firstpoint, const uint16_t &finalID)
{
  if (fabs(last_first_x - firstpoint.x) >= 0.05 && fabs(last_first_y - firstpoint.y) >= 0.05)
  {
    LOG(INFO) << "[Check extend path] NG";
    LOG(INFO) << "current first point (" << firstpoint.x << ", " << firstpoint.y << ")"
               << "\nlast first point (" << last_first_x << ", " << last_first_y << ")";
    printf("ng 1\n");
    return false;
  }
  if (lastfinalgoalid != finalID)
  {
    LOG(INFO) << "[Check extend path] NG";
    LOG(INFO) << "current final goal:" << finalID << ", last final goal:" << lastfinalgoalid;
    printf("ng 2\n");
    return false;
  }
  LOG(INFO) << "[Check extend path] OK";
  return true;
}

double SpeedAdjuster_(double currentSpeed,
                      const double &targetSpeed,
                      const double &deceleration,
                      bool minimum_speed_set = false,
                      double minimum_speed = 0)
{
  currentSpeed -= Sign(currentSpeed) * deceleration / looprate;
  if (targetSpeed * currentSpeed > 0)
  {
    if (minimum_speed_set)
    {
      if (fabs(targetSpeed) <= minimum_speed &&
          ((targetSpeed < 0 && fabs(currentSpeed) <= fabs(targetSpeed)) ||
           (targetSpeed > 0 && currentSpeed < targetSpeed)))
        currentSpeed = targetSpeed;
      else if (fabs(currentSpeed) <= minimum_speed && fabs(targetSpeed) >= minimum_speed)
        currentSpeed = Sign(currentSpeed) * minimum_speed;
    }
    else
    {
      if ((targetSpeed < 0 && fabs(currentSpeed) < fabs(targetSpeed)) ||
          (targetSpeed > 0 && currentSpeed < targetSpeed))
        currentSpeed = targetSpeed;
    }
  }
  return currentSpeed;
}


void SpeedAdjuster(const double &targetRoV, const int &type)//20210203wusl Keep this function
{
  if (targetRoV == current_cmd_vel.linear.x)
    current_cmd_vel.linear.x = targetRoV;
  else if (targetRoV == 0)
  {
    current_cmd_vel.linear.x -= Sign(current_cmd_vel.linear.x) * veldecel_emergency / looprate;
    if (fabs(current_cmd_vel.linear.x) <= veldecel_emergency / looprate)
      current_cmd_vel.linear.x = 0;
  }
  else
  {
    switch (type)
    {
    case 0:
      if (isSlowDown || isErrorSlowDown)
      {
        if (fabs(current_cmd_vel.linear.x) < vel_low && fabs(current_cmd_vel.linear.x) < fabs(targetRoV))
        {
          current_cmd_vel.linear.x += Sign(targetRoV) * velacel / looprate;
          if (fabs(current_cmd_vel.linear.x) > fabs(targetRoV))
            current_cmd_vel.linear.x = targetRoV;
          else if (fabs(current_cmd_vel.linear.x) >= vel_low)
            current_cmd_vel.linear.x = Sign(current_cmd_vel.linear.x) * vel_low;
        }
        else
        {
          current_cmd_vel.linear.x = SpeedAdjuster_(current_cmd_vel.linear.x, targetRoV, 2.0 * veldecel, true, vel_low);
        }
      }
      else if (fabs(current_cmd_vel.linear.x) < fabs(targetRoV))
      {
        current_cmd_vel.linear.x += Sign(targetRoV) * velacel / looprate;
        if (fabs(current_cmd_vel.linear.x) >= vel_max)
          current_cmd_vel.linear.x = Sign(current_cmd_vel.linear.x) * vel_max;
        else if (fabs(current_cmd_vel.linear.x) > fabs(targetRoV))
          current_cmd_vel.linear.x = targetRoV;
      }
      else
      {
        current_cmd_vel.linear.x = SpeedAdjuster_(current_cmd_vel.linear.x, targetRoV, veldecel);
      }
      break;
    case 1:
      if (fabs(current_cmd_vel.linear.x) < fabs(targetRoV))
      {
        current_cmd_vel.linear.x += Sign(targetRoV) * velacel_secondary / looprate;
        if (fabs(current_cmd_vel.linear.x) >= vel_Secondary)
          current_cmd_vel.linear.x = Sign(current_cmd_vel.linear.x) * vel_Secondary;
      }
      else
      {
        current_cmd_vel.linear.x = SpeedAdjuster_(current_cmd_vel.linear.x, targetRoV, velacel_secondary);
      }
      break;
    default:
      LOG(INFO) << "Wrong tracetype!";
      current_cmd_vel.linear.x = 0;
      break;
    }
  }
}

//20210203wusl add start
void SpeedAdjusterX(const double &targetRoV, const int &type)//20210203wusl
{
  if (targetRoV == current_cmd_vel.linear.x)
    current_cmd_vel.linear.x = targetRoV;
  else if (targetRoV == 0)
  {
    current_cmd_vel.linear.x -= Sign(current_cmd_vel.linear.x) * veldecel_emergency / looprate;
    if (fabs(current_cmd_vel.linear.x) <= veldecel_emergency / looprate)
      current_cmd_vel.linear.x = 0;
  }
  else
  {
    switch (type)
    {
    case 0:
      if (isSlowDown || isErrorSlowDown)
      {
        if (fabs(current_cmd_vel.linear.x) < vel_low && fabs(current_cmd_vel.linear.x) < fabs(targetRoV))
        {
          current_cmd_vel.linear.x += Sign(targetRoV) * velacel / looprate;
          if (fabs(current_cmd_vel.linear.x) > fabs(targetRoV))
            current_cmd_vel.linear.x = targetRoV;
          else if (fabs(current_cmd_vel.linear.x) >= vel_low)
            current_cmd_vel.linear.x = Sign(current_cmd_vel.linear.x) * vel_low;
        }
        else
        {
          current_cmd_vel.linear.x = SpeedAdjuster_(current_cmd_vel.linear.x, targetRoV, 2.0 * veldecel, true, vel_low);
        }
      }
      else if (fabs(current_cmd_vel.linear.x) < fabs(targetRoV))
      {
        current_cmd_vel.linear.x += Sign(targetRoV) * velacel / looprate;
        if (fabs(current_cmd_vel.linear.x) >= vel_max)
          current_cmd_vel.linear.x = Sign(current_cmd_vel.linear.x) * vel_max;
        else if (fabs(current_cmd_vel.linear.x) > fabs(targetRoV))
          current_cmd_vel.linear.x = targetRoV;
      }
      else
      {
        current_cmd_vel.linear.x = SpeedAdjuster_(current_cmd_vel.linear.x, targetRoV, veldecel);
      }
      break;
    case 1:
      if (fabs(current_cmd_vel.linear.x) < fabs(targetRoV))
      {
        current_cmd_vel.linear.x += Sign(targetRoV) * velacel_secondary / looprate;
        if (fabs(current_cmd_vel.linear.x) >= vel_Secondary)
          current_cmd_vel.linear.x = Sign(current_cmd_vel.linear.x) * vel_Secondary;
      }
      else
      {
        current_cmd_vel.linear.x = SpeedAdjuster_(current_cmd_vel.linear.x, targetRoV, velacel_secondary);
      }
      break;
    default:
      LOG(INFO) << "Wrong tracetype!";
      current_cmd_vel.linear.x = 0;
      current_cmd_vel.linear.y = 0; //20210203wusl
      break;
    }
  }
}
//20210203wusl add end

//20210203wusl add start
void SpeedAdjusterY(const double &targetRoV, const int &type)//20210203wusl
{
  if (targetRoV == current_cmd_vel.linear.y)
    current_cmd_vel.linear.y = targetRoV;
  else if (targetRoV == 0)
  {
    current_cmd_vel.linear.y -= Sign(current_cmd_vel.linear.y) * veldecel_emergency / looprate;
    if (fabs(current_cmd_vel.linear.y) <= veldecel_emergency / looprate)
      current_cmd_vel.linear.y = 0;
  }
  else
  {
    switch (type)
    {
    case 0:
      if (isSlowDown || isErrorSlowDown)
      {
        if (fabs(current_cmd_vel.linear.y) < vel_low && fabs(current_cmd_vel.linear.y) < fabs(targetRoV))
        {
          current_cmd_vel.linear.y += Sign(targetRoV) * velacel / looprate;
          if (fabs(current_cmd_vel.linear.y) > fabs(targetRoV))
            current_cmd_vel.linear.y = targetRoV;
          else if (fabs(current_cmd_vel.linear.y) >= vel_low)
            current_cmd_vel.linear.y = Sign(current_cmd_vel.linear.y) * vel_low;
        }
        else
        {
          current_cmd_vel.linear.y = SpeedAdjuster_(current_cmd_vel.linear.y, targetRoV, 2.0 * veldecel, true, vel_low);
        }
      }
      else if (fabs(current_cmd_vel.linear.y) < fabs(targetRoV))
      {
        current_cmd_vel.linear.y += Sign(targetRoV) * velacel / looprate;
        if (fabs(current_cmd_vel.linear.y) >= vel_max)
          current_cmd_vel.linear.y = Sign(current_cmd_vel.linear.y) * vel_max;
        else if (fabs(current_cmd_vel.linear.y) > fabs(targetRoV))
          current_cmd_vel.linear.y = targetRoV;
      }
      else
      {
        current_cmd_vel.linear.y = SpeedAdjuster_(current_cmd_vel.linear.y, targetRoV, veldecel);
      }
      break;
    case 1:
      if (fabs(current_cmd_vel.linear.y) < fabs(targetRoV))
      {
        current_cmd_vel.linear.y += Sign(targetRoV) * velacel_secondary / looprate;
        if (fabs(current_cmd_vel.linear.y) >= vel_Secondary)
          current_cmd_vel.linear.y = Sign(current_cmd_vel.linear.y) * vel_Secondary;
      }
      else
      {
        current_cmd_vel.linear.y = SpeedAdjuster_(current_cmd_vel.linear.y, targetRoV, velacel_secondary);
      }
      break;
    default:
      LOG(INFO) << "Wrong tracetype!";
      current_cmd_vel.linear.x = 0; //20210203wusl
      current_cmd_vel.linear.y = 0;
      break;
    }
  }
}
//20210203wusl add end

void SpeedAdjuster(const double &targetRoW)
{
  // robdir = 0 is used to adjust when going straight, avoid
  if (targetRoW == current_cmd_vel.angular.z || robdir == 0)
    current_cmd_vel.angular.z = targetRoW;
  else if (targetRoW == 0)
  {
    current_cmd_vel.angular.z -= 1.2 * Sign(current_cmd_vel.angular.z) * wanacel / looprate;
    if (fabs(current_cmd_vel.angular.z) <= wanacel / looprate)
      current_cmd_vel.angular.z = 0;
  }
  else if (isSlowDown)
  {
    if (fabs(current_cmd_vel.angular.z) < wan_low && fabs(current_cmd_vel.angular.z) < fabs(targetRoW))
    {
      current_cmd_vel.angular.z += Sign(targetRoW) * wanacel / looprate;
      if (fabs(current_cmd_vel.angular.z) > fabs(targetRoW))
        current_cmd_vel.angular.z = targetRoW;
      else if (fabs(current_cmd_vel.angular.z) >= wan_low)
        current_cmd_vel.angular.z = Sign(current_cmd_vel.angular.z) * wan_low;
    }
    else
    {
      current_cmd_vel.angular.z = SpeedAdjuster_(current_cmd_vel.angular.z, targetRoW, 1.2 * wanacel, true, wan_low);
    }
  }
  else if (fabs(current_cmd_vel.angular.z) < fabs(targetRoW))
  {
    current_cmd_vel.angular.z += Sign(targetRoW) * wanacel / looprate;
    if (fabs(current_cmd_vel.angular.z) > wan_max)
      current_cmd_vel.angular.z = Sign(current_cmd_vel.angular.z) * wan_max;
    else if (fabs(current_cmd_vel.angular.z) > fabs(targetRoW))
      current_cmd_vel.angular.z = targetRoW;
  }
  else
  {
    current_cmd_vel.angular.z = SpeedAdjuster_(current_cmd_vel.angular.z, targetRoW, wanacel);
  }
}

void SpeedAdjuster(const geometry_msgs::Vector3 &target_linear_vel, const double &targetRoW, const int &type)
{
  //SpeedAdjuster(target_linear_vel.x, type);//20210201wusl
  SpeedAdjusterX(target_linear_vel.x, type);//20210201wusl
  SpeedAdjusterY(target_linear_vel.y, type); //20210201wusl
  SpeedAdjuster(targetRoW);
}

void result_callBack(const move_base_msgs::MoveBaseActionResult &msg)
{
  ROS_INFO("[SLAM] status=%d",msg.status.status);
  /* 210208jimc if(msg.status.status==3) goal_result =1; */
  /* 210208jimc add start */
  if(msg.status.status==3) 
  {
    goal_result = 1;
    abortcount = 0;
  }
  /* 210208jimc add end */
  else if(msg.status.status==4)
  {
    /* 210208jimc ROS_INFO("[SLAM]Abort"); */
    /* 210208jimc AlarmCode = 12; */
    /* 210208jimc add start */
    abortcount+=1;
    if( abortcount == 1)    //The normal navigation path cannot continue,UI chiose whether avoid path
    {
      pausestop = 0; /* 211012jimc */
      /* 210401jimc add */
      ROS_INFO("obstacle_deceleration is working,path cannot continue");  

      if( ( nav_param_select == 0 ) || ( nav_param_select == 2) )
      {
        AlarmCode = 999;
        ROS_INFO("[SLAM]Abort:The normal navigation path cannot continue,UI chiose none avoid path");
      }
      else
      {
        ROS_ERROR("reset pauestop");
        change_nav_param(1);  //change to void path parameter /* 210407jimc */
        nav_param_select_previou = 1;                         /* 210924jimc */
        switch_flag=1;
        replan_goal_flag = 1; //chang move mode  or back to last point 0323
        obsdec_avoid_flg = 1;
        planner_costmap_ros_->getRobotPose(obsdec_lastgoal_pose);
      }
      /* 210401jimc add end */
      //AlarmCode = 91;
      //ROS_INFO("[SLAM]Abort:The normal navigation path cannot continue,UI chiose whether avoid path");
    }
    if( abortcount == 2)    //The obstacle avoidance path cannot continue,UI decide back to previous point
    {
      //replan_goal_flag=1;
      abortcount = 0;
      AlarmCode = 999;  //stop at now point /* 210401jimc */
      ROS_INFO("[SLAM]Abort:The obstacle avoidance path cannot continue,UI decide back to previous point");
    }
    /*
    if( abortcount == 3)    //Still cannot move,need operator get involved
    {
      abortcount = 0;
      AlarmCode = 91;
      ROS_INFO("[SLAM]Abort:Still cannot move,need operator get involved");
    }*/
  
   /* 210208jimc add end */
  }
}

void slam_cmd_vel_callBack(const geometry_msgs::Twist &cmd_vel)
{
  //printf("1");
  get_slam_cmd = 1;
  /* 210518jimc if(!(pausestop || remathstop)  ) slam_cmd_vel = cmd_vel; ////0331 add !remathstop */
  /* 210518jimc add */
  if( !(pausestop || remathstop) )
  {
    slam_cmd_vel = cmd_vel; 
  }
  /* 210518jimc add end */
  else
  {
    slam_cmd_vel.linear.x = 0; 
    slam_cmd_vel.linear.y = 0;//20210201wusl
    slam_cmd_vel.angular.z = 0;  
  }
  
}
//0427 char 
void robot_pose_now_callBack(const geometry_msgs::Pose &pose_tmp)
{
  switch_map_inf.request.location.pose.orientation = pose_tmp.orientation; /* 210421 char */
}
void TEB_path_tf_callBack(const geometry_msgs::Pose &pose_tmp)//1104 char  /* teb_path */
{
  TEB_path_pose =pose_tmp;
}
/* 210511GU add */
void bay_response_callback(const std_msgs::Int16::ConstPtr &data)
{
  bay_flag = data->data;
}
/* 210511GU add end */
//0927 char
void velctrl_callback(const std_msgs::Int32::ConstPtr &data)
{
  velctrl_flag =data->data;
}

void laser_callback(const sensor_msgs::LaserScan::ConstPtr &data)  //2021GU
{
  int range = data->ranges.size();     
  double degree = data->angle_increment;  //each point degree(rad)
  //double left_dis[360] = {0};
  //double right_dis[360] = {0}; 
  std::vector<double> left_dis;
  std::vector<double> right_dis;
  left_dis.clear();
  right_dis.clear();
  left_dis.push_back(0);
  right_dis.push_back(0);

  double laser_left_thres=0, laser_right_thres=0;
  for(int i=0;i<360;i++)
  {
   int point1 = int(range/8*5)+i;  //left
   //left_dis[i] = (data->ranges[point1]) * fabs(sin(degree*point1));
   if(data->ranges[point1]<= 0.75)
     {left_dis.push_back((data->ranges[point1]) * fabs(sin(degree*point1)));}
   else if(data->ranges[point1]>0.75 && data->ranges[point1]<3.0)
     {laser_left_thres++;}

   int point2 = int(range/8*2)+i;   //right
   //right_dis[i] = (data->ranges[point2]) * sin(degree*point2);
   if(data->ranges[point2] <= 0.75)
     {right_dis.push_back((data->ranges[point2]) * sin(degree*point2));}
   else if(data->ranges[point2]>0.75 && data->ranges[point2]<3.0)
     {laser_right_thres++;}
  }   
 
  int left_size = left_dis.size();
  int right_size = right_dis.size();

  std::sort(left_dis.begin(), left_dis.begin() + left_size);
  std::sort(right_dis.begin(), right_dis.begin() + right_size);

  /*printf("L:");
  for(int i=0;i<left_size;i++)
   {printf("%.3f ",left_dis[i]);}
  printf("\n");*/

  /*printf("R:");
  for(int i=0;i<right_size;i++)
   {printf("%.3f ",right_dis[i]);}
  printf("\n");*/

  //add array total
  int lasercount1=0, lasercount2=0;
  double total_L=0.0, total_R=0.0;
  for(int i=0;i<left_size;i++)
  {
      if(left_dis[i]!=0.0 && lasercount1<10)
       {
         total_L = total_L + left_dis[i];
         lasercount1++;
       }
  }
  
  if(laser_left_thres>180 || total_L==0)
     laser_left = -1;
  else
     laser_left = total_L/lasercount1;

  for(int i=0;i<right_size;i++)
  {
      if(right_dis[i]!=0.0 && lasercount2<10)
       {
         total_R = total_R + right_dis[i];
         lasercount2++;
       }
  }
  //printf("total:%.3f count:%d\n",total_R,lasercount2);
  if(laser_right_thres>180 || total_R==0)
     laser_right = -1;
  else
     laser_right = total_R/lasercount2;
 
  //printf("[Left]:%.3f\n",laser_left);
  //printf("[Right]:%.3f\n",laser_right);
}
/* 210205jimc add start */
// mode_select (0:Non use void parameter, 1:Use void parameter, 2:Enter Bay parameter)
void change_nav_param(int mode_select)
{
  LOG(INFO) << "mode_select : " << mode_select;
  ROS_INFO("mode_select : %d " , mode_select);
  std_msgs::Int32 mode;
  mode.data =0;  //0927
  velctrl_flag =0;
  if( mode_select == 0 )        // Main road non use void parameter
  {
    //costmap hasn't obst
    system("rosrun dynamic_reconfigure dynparam load /move_base/global_costmap/obstacle_layer /home/gpm/catkin_ws/src/gpm_project/barcodemove/param/mode0_not_obst_costmap.yaml");
    //teb
    system("rosrun dynamic_reconfigure dynparam load /move_base/TebLocalPlannerROS /home/gpm/catkin_ws/src/gpm_project/barcodemove/param/mode0_teb_not_aovidance.yaml");  
    
    system("rosrun dynamic_reconfigure dynparam load /move_base/local_costmap/inflation_layer /home/gpm/catkin_ws/src/gpm_project/barcodemove/param/mode0_local_costmap_inflation.yaml");  

    LOG(INFO) << "Change Non use void parameter";
    ROS_INFO("Change Non use void parameter");
  }
  else if ( mode_select == 1 )  // Main road use void parameter
  {
    //costmap has obst
    system("rosrun dynamic_reconfigure dynparam load /move_base/global_costmap/obstacle_layer /home/gpm/catkin_ws/src/gpm_project/barcodemove/param/mode1_obst_costmap.yaml");

    system("rosrun dynamic_reconfigure dynparam load /move_base/TebLocalPlannerROS /home/gpm/catkin_ws/src/gpm_project/barcodemove/param/mode1_teb_nav_barcodemove_slide.yaml");           
    
    LOG(INFO) << "Change Use void parameter";
    ROS_INFO("Change Use void parameter");
    mode.data=2;//0930 char 
  }
  else if ( mode_select == 2 )  // Enter Bay parameter non use void parameter
  {
    //global planner
    system("rosrun dynamic_reconfigure dynparam load /move_base/GlobalPlanner /home/gpm/catkin_ws/src/gpm_project/barcodemove/param/mode2_global_planner_bay.yaml");
    //teb
    system("rosrun dynamic_reconfigure dynparam load /move_base/TebLocalPlannerROS /home/gpm/catkin_ws/src/gpm_project/barcodemove/param/mode2_teb_nav_bay.yaml");
    //costmap hasn't obst
    system("rosrun dynamic_reconfigure dynparam load /move_base/global_costmap/obstacle_layer /home/gpm/catkin_ws/src/gpm_project/barcodemove/param/mode2_not_obst_costmap.yaml");
    
    system("rosrun dynamic_reconfigure dynparam load /move_base/local_costmap/inflation_layer /home/gpm/catkin_ws/src/gpm_project/barcodemove/param/mode2_local_costmap_inflaton.yaml");  

    LOG(INFO) << "Change Enter Bay parameter";
    ROS_INFO("Change Enter Bay parameter");

  }
  else
  {
    LOG(INFO) << "Dynparam not change mode_select input error";
    ROS_INFO("Dynparam not change mode_select input error");
  }
  obst_mode_pub.publish(mode);
}
/* 210205jimc add end */
/* 210324jimc add */
void velCallback(const geometry_msgs::Twist &vel_tmp)
{
	bool frontL = 0, frontR = 0, backL = 0, backR = 0;
  if( artagexe != 1)                 /* 210423jimc */
  {
    robdir = 0;      /* 210429jimc */
    //0524 char change 0 to 0.1
    if(    
           ( fabs(vel_tmp.linear.x) <= 0.1 )
        && ( fabs(vel_tmp.linear.y) <= 0.1 )
    )
    {
        robdir = 0;
    }
    //forward
    /* 210511jimc mark
    if(vel_tmp.linear.x > 0)
    {
      frontL = 1;
      frontR = 1;
      robdir = 0;
    }
    //backward
    if(vel_tmp.linear.x < 0)
    {
      backL = 1;
      backR = 1;
      robdir = 4;
    }
    210511jimc mark end */

    //drift left
    if(vel_tmp.linear.y > 0.1)
    {
      frontL = 1;
      backL = 1;
      robdir = 1;
      //ROS_INFO("linear y +" );
    }
    //drift right
    if(vel_tmp.linear.y < -0.1)
    {
      frontR = 1;
      backR = 1;
      robdir = 2;
      //ROS_INFO("linear y -" );
    }
    //turn left
    if(vel_tmp.angular.z >= 0.03)//0  //20210430
    {
      frontL = 1;
      backR = 1;
      robdir = 1;
      //ROS_INFO("linear Z +" );
    }
    //turn right
    if(vel_tmp.angular.z <= -0.03)//0  //20210430
    {
      frontR = 1;
      backL = 1;
      robdir = 2;
      //ROS_INFO("linear Z -" );
    }
    
    /* 210408jimc add */
    // obstacle
    /* 210518jimc if(obsdecState) */
    if( pausestop == 1 )   /* 210518jimc */
    {
      robdir = 99;
    }
    /* 210408jimc add end */

    /* 210428jimc add */
    if(instrumentState)
    {
      robdir = 0;
    }
    /* 210428jimc add end */
  }
  else
  {
      robdir = 3;  /* 210423jimc */
  }

	//左前右前左後右後0,1,2,3
	//ROS_INFO("frontL=%d, frontR=%d, backL=%d, backR=%d", frontL, frontR, backL, backR);
  //ROS_INFO("robdir=%d ", robdir);
}
/* 210324jimc add end */

//action server thread call back
void execute(const gpm_msgs::TaskCommandGoalConstPtr &goal, Server *as)
{
  ros::NodeHandle n;
  ros::Rate r(looprate);
  ros::Rate r1(1);  /* 210429jimc */
  
  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
     ROS_INFO("Waiting for the move_base action server to come up");
  }
  
  gpm_msgs::TaskCommandFeedback feedback;

  std_msgs::Int16 bay_request;                                                  /* 210511GU */

  nav_msgs::Path cglobal_plan = goal->planPath;
  /* to compare with the extent path */
  geometry_msgs::Point firstPoint;
  /* taks type, follow, forward docking, backward docking */
  uint8_t tracetype = goal->mobilityModes;
  /* dynpath, get global final id */
  uint16_t finalgoalid = goal->finalGoalID;
  /* the numbers of received path */
  int goal_nums = cglobal_plan.poses.size();

  /* 210311jimc add start */
  /* Slam navigation path information */
  gpm_msgs::PathInfo plan_pathinfo[goal_nums];
  if( goal->pathInfo.size() > 0 )
  {
    for( int i = 0; i < goal_nums ; i++ )
    {
      plan_pathinfo[i] = goal->pathInfo[i];
      ROS_INFO("pathinfo[%d].tagid : %d" , i,plan_pathinfo[i].tagid);
      ROS_INFO("pathinfo[%d].laserMode : %d" , i,plan_pathinfo[i].laserMode);
      ROS_INFO("pathinfo[%d].direction : %d" , i,plan_pathinfo[i].direction);
      ROS_INFO("pathinfo[%d].map : %s" , i,plan_pathinfo[i].map.c_str());
      ROS_INFO("pathinfo[%d].changemap : %d" , i,plan_pathinfo[i].changeMap);
    }
  }
  /* 210311jimc add end */

  tf::Stamped<tf::Pose> grobot_pose;
  tf::Stamped<tf::Pose> lastgoal_pose;
  tf::Stamped<tf::Pose> nextgoal_pose;
  tf::Stamped<tf::Pose> subfinalgoal_pose;
  tf::Stamped<tf::Pose> p0;
  tf::Stamped<tf::Pose> p1;
  tf::Stamped<tf::Pose> p2;
  geometry_msgs::Twist cmd_vel;
  std::vector<int> segmentid;
  std::vector<int> bends;
  std::vector<geometry_msgs::PoseStamped> segment_plan;
  std::vector<gpm_msgs::PathInfo> segment_plan_pathinfo; /* 210311jimc */

  /* 210205jimc add start */
  artag_msgs::artag_path ar_tag_sendTagID;   /* Send current & goal ID to AR tag */
  std_msgs::Bool ar_tag_move_flag;           /* true: start ,false: not ready */
  std_msgs::UInt8 ar_tag_func_flag;          /* 0:Normal, 1:Stop, 2:Reset */
  std_msgs::UInt8 agv_modbus_goalID;         /* 210305jimc */

  /* 210205jimc add end */
  float robot_ang;                   // current robot angle in global map
  double nextdist_err;               // current robot point to next target point distance error
  double tempdist_err;
  float nextangle_err;               // current robot angle to next target point angle error
  double robtolindis_err;            // current robot point to a line distance error
  float robtolinang_err;             // current robot angle vs the line angle error
  float finalroangle_err;            // current robot angle vs the final robot angle error
  double finalrodist_err;            // cal point to point or robot pose distance
  double robotfwdist_err;            // robot forward to target distance project to line
  geometry_msgs::Vector3 linear_vel; // robot forward velocity
  double RoW;                        // robot angular velocity
  int trackingErrorCount = 0;
  int subgoal_nums; // the numbers of sub segment path
  //tracking path flag
  bool anglefw = 0;
  bool positionon = 0;
  bool angleon = 0;
  bool nearflag = 0;
  bool vel_cancel_pub = false;    /* 210511GU */

  //action server status flag
  bool preemptreq = false;
  bool emptycancel = false;
  bool gettag_vir = false;
  int segno = 0;
  int segid = 0;
  int subid = 0;
  int isFindingTag_count = 0; //initial it when get new task
  float angleerr;
  int pose_count = 0;	//test check pose on tag < 10mm
  /* 210205jimc add start */
  // nav_param_select = -1;
  nav_param_select_previou = -1;
  /* 210205jimc add end */

  /* 210407jimc add */
  abortcount = 0;   
  back_flag = 0;
  pausestop = 0;
  ROS_WARN("reset at start");
  isSlowDown = false;
  vel_max = speedmax;
  /* 210407jimc add end */
  artagexe = 0;                 /* 210423jimc */
  pass_point_check = false;     /* 210913jimc */
  
  if (forwardstop == 1)
  {
    LOG(INFO) << "receive forward stop after the end of the loop, so clear all flags";
    /* 210409jimc forwardstop = 0; */
    forwardstate = 1;
    dynflag = lastdynflag = 0;
    lastsegid = lastsubid = 0;
    current_cmd_vel.linear.x=current_cmd_vel.linear.y = current_cmd_vel.angular.z = 0;
  }
  LOG(INFO) << "[Goal info] goal no: " << goal_nums << " trace type: " << (int)tracetype;
  ROS_INFO("[Goal info] goal no:  %d  trace type:  %d " ,goal_nums ,(int)tracetype); //show info
  ROS_INFO("get new task");
  //pausestop=0;//0330
  //publish received path
  plan_pub.publish(cglobal_plan);
  //check a new task within tag in nav path type
  //if last state is forwardstop, it may not locate in tag for next new task
  //if last task is local path, it may not locate in tag for next new extended path

  //ROS_INFO("tracetype:%d goal_nums:%d forwardstate:%d lastdynflag:%d",tracetype,goal_nums,forwardstate,lastdynflag);
  if (tracetype == 0 && goal_nums >= 1 )
  {
    if (forwardstate == 1)
      forwardstate = 0;
    isRoughAdjust = false;
    //0316 want to check
    firstPoint = cglobal_plan.poses[0].pose.position;
    dynflag = IsCompletedTask(cglobal_plan.poses[goal_nums - 1].header.seq, finalgoalid);   //check is completed (0:completed ,1:uncompleted)
    AlarmCode = 0;
    segmentid.clear();
    segment_plan.clear();
    segment_plan_pathinfo.clear(); /* 210311jimc */
    segmentid.push_back(0);
    //loop to find po  p1  p2 , find the corner index keep in [segmentid]
    //push corner angle push back to the point in [cglobal_plan]
    float line1ang, line2ang, angleerr;
    for (int i = 1; i < goal_nums - 1; i++)
    {
      poseStampedMsgToTF(cglobal_plan.poses[i - 1], p0);
      poseStampedMsgToTF(cglobal_plan.poses[i], p1);
      poseStampedMsgToTF(cglobal_plan.poses[i + 1], p2);

      #if 0 /* 210923jimc mark */
      double p1_p2_x_dis = p1.getOrigin().x() - p2.getOrigin().x();
      double p1_p2_y_dis = p1.getOrigin().y() - p2.getOrigin().y();
      double p0_p1_x_dis = p0.getOrigin().x() - p1.getOrigin().x();
      double p0_p1_y_dis = p0.getOrigin().y() - p1.getOrigin().y();

      if ((fabs(p1_p2_x_dis) > 0.1 && fabs(p1_p2_y_dis) > 0.1) ||
          (fabs(p0_p1_x_dis) > 0.1 && fabs(p0_p1_y_dis) > 0.1 && i == 1))
      {
        segmentid.push_back(i);
        //cglobal_plan.poses[i].pose.orientation = cglobal_plan.poses[i + 1].pose.orientation;
        ROS_INFO("[Get corner] get corner id (different point): %d ,z: %f , w: %f " ,cglobal_plan.poses[i].header.seq
                  ,cglobal_plan.poses[i].pose.orientation.z , cglobal_plan.poses[i].pose.orientation.w);
        
        LOG(INFO) << "[Get corner] get corner id (different point): " << cglobal_plan.poses[i].header.seq
                  << " z: " << cglobal_plan.poses[i].pose.orientation.z << " w: " << cglobal_plan.poses[i].pose.orientation.w;
        continue;
      }
      #endif  /* 210923jimc mark end */
      
      segmentid.push_back(i);   /* 210923jimc */
      line1ang = CalculateLineAngle(p1, p0);
      line2ang = CalculateLineAngle(p1, p2);
      angleerr = line2ang - line1ang;
      angleerr = atan2(sin(angleerr), cos(angleerr));
      ROS_INFO("angleerr = %f",(angleerr *180 / M_PI));/* 210923jimc */
       //find a corner 0427
      /* 210923jimc if (fabs(angleerr) < 0.98 * M_PI)//0.98 */
      if (fabs(angleerr) < ( (180 - 20) * M_PI / 180 ) )     /* 210923jimc */
      {
        /* 210923jimc segmentid.push_back(i); */
        corner_flag =1;
        plan_pathinfo[i].corner = 1;  /* 210901jimc */
        ROS_INFO("find corner");
        //quaternionTFToMsg(tf::createQuaternionFromYaw(line2ang), cglobal_plan.poses[i].pose.orientation);
        ROS_INFO("[Get corner] get corner id: %d , z: %f , w: %f  " , cglobal_plan.poses[i].header.seq
                  ,cglobal_plan.poses[i].pose.orientation.z , cglobal_plan.poses[i].pose.orientation.w );
        LOG(INFO) << "[Get corner] get corner id: " << cglobal_plan.poses[i].header.seq
                  << " z: " << cglobal_plan.poses[i].pose.orientation.z << " w: " << cglobal_plan.poses[i].pose.orientation.w;
      }
      else
      {
          corner_flag =0; //0427
        plan_pathinfo[i].corner = 0;  /* 210901jimc */
      }
    }
    segmentid.push_back(goal_nums - 1);
    segno = segmentid.size() - 1;
	  //printf("segno=%d\n",segno);
    ROS_INFO("[Segment info] segment nums: %d " ,segno);
    LOG(INFO) << "[Segment info] segment nums:" << segno;
    //dynpath , check if a new path with different finalgoalid
    if (lastdynflag == 1)
    {
      segid = lastsegid;
      subid = lastsubid;
      if (!IsCorrectExtentPath(firstPoint, finalgoalid))
        AlarmCode = 2;  //WrongExtendPath
    }
    else
    {
      segid = 0;
      subid = IsAGVOnLineDirection(cglobal_plan) ? 1 : 0; //let it skip tracking initial point
      planner_costmap_ros_->getRobotPose(grobot_pose);
      LOG(INFO) << "first point: " << firstPoint.x << ", " << firstPoint.y;
      LOG(INFO) << "current position: " << grobot_pose.getOrigin().x() << ", " << grobot_pose.getOrigin().y();
      //0316 check
      printf("x %f ,y %f\n",firstPoint.x,firstPoint.y);

      if( forwardstop != 1 )  /* 210409jimc */
      {
        if (fabs(firstPoint.x - grobot_pose.getOrigin().x()) > 2 ||
            fabs(firstPoint.y - grobot_pose.getOrigin().y()) > 2)
        {
          //0316 check
          printf("x_err %f y_err %f\n",fabs(firstPoint.x - grobot_pose.getOrigin().x()),fabs(firstPoint.y - grobot_pose.getOrigin().y()));
          LOG(INFO) << "AGV is too far from the first point";
          ROS_INFO("AGV is too far from the first point");
          AlarmCode = 2;  //WrongExtendPath
        }
      }
      else
      {
        forwardstop = 0;  /* 210409jimc */
      }
    }

    LOG(INFO) << "[Loop Info] dynflag: (Now)" << dynflag << ", (last)" << lastdynflag;
    LOG(INFO) << "[Loop Info] now-segid: " << segid << ", now-subid: " << subid;
    //1 segment loop
    for (int s = segid; s < segno; s++)
    {
	    printf("s=%d segno=%d\n",s,segno);
      if( (AlarmCode != 0) || preemptreq)
        break;
      lastsegid = s;
      segment_plan.clear();
      segment_plan_pathinfo.clear(); /* 210311jimc */
      //copy a segment from cglobal_plan
      for (int p = segmentid[s]; p <= segmentid[s + 1]; p++)
      {
        segment_plan.push_back(cglobal_plan.poses[p]);
        segment_plan_pathinfo.push_back(plan_pathinfo[p]); /* 210311jimc */
        LOG(INFO) << "[LoopSeg] passing id: " << cglobal_plan.poses[p].header.seq<<" x:"<<cglobal_plan.poses[p].pose.position.x<<
          " y:"<<cglobal_plan.poses[p].pose.position.y<<" z:"<<cglobal_plan.poses[p].pose.orientation.z<<" w:"<<cglobal_plan.poses[p].pose.orientation.w;
      }

      subgoal_nums = segment_plan.size();
      LOG(INFO) << "[LoopSeg] seg:" << s + 1 << "/" << segno << ", subid:" << subid << "~" << subgoal_nums - 1;
      poseStampedMsgToTF(segment_plan[subgoal_nums - 1], subfinalgoal_pose);
      LOG(INFO)<<"subfinalgoal_pose "<<segment_plan[subgoal_nums - 1].header.seq;

      //2 subitem loop in a segment
      for (int i = subid; i < subgoal_nums; i++)
      {
        if( (AlarmCode != 0) || preemptreq)
          break;
        if (i >= subid)
          subid = 0;
        lastsubid = i;
        anglefw = false;
        positionon = false;
        angleon = false;
        nearflag = false;
        trackingErrorCount = 0;

        int nextID = segment_plan[i].header.seq; //0115
        // tracking path looping
        LOG(INFO) << "i=" << i << "move to" << nextID;
	      printf("i=%d move to %d\n",i,nextID);

        misstion_exe_flg = 0;             /* 210401jimc */

	      bool set_goal = 0;
        //check tag start
        bool check_tag = 0;
        int check_tag_cnt = 0; //0119
        bool tagangon = 0;
        int tag_stable_cnt = 0;
        //check tag end

        //20210111wusl add start
        int iSmooth_Step=99;                                 //20210111wusl
        bool bInSmooth=false;                                //20210111wusl
        double dSmooth_time_start = ros::Time::now().toSec();//20210111wusl
        //############################
        goalRange1 = 0.3; //0.2
        goalRange2 = 0.5;
        goalRange3 = 0.7;

        goalSpeed1 = 0.5;//0.3;
        goalSpeed2 = 0.6;
        goalSpeed3 = 0.8;
        //###########################
        //20210111wusl add end

        /* 210409jimc add */
        if( segment_plan_pathinfo[i].changeMap == 2 )
        {
          cmd_vel.linear.x = cmd_vel.linear.y = cmd_vel.angular.z = 0;
          velocity_pub.publish(cmd_vel);

          switch_map_name = segment_plan_pathinfo[i].map.c_str();

          if( switch_map_name != pre_switch_map_name )
          {
            /* 210421 char add */
            std_msgs::Bool map_switch;
            map_switch.data = true;
            map_switch_pub.publish(map_switch);
            /* 210421 char add end */
            
            pre_switch_map_name = switch_map_name;
            LOG(INFO) << "Switch_map to : " << switch_map_name;
            ROS_INFO("Switch_map to : %s" , switch_map_name.c_str());
            
            LOG(INFO) << "Initial pose X : " << segment_plan[i].pose.position.x;
            ROS_INFO("Initial pose X : %.4f " , segment_plan[i].pose.position.x);

            LOG(INFO) << "Initial pose Y : " << segment_plan[i].pose.position.y;
            ROS_INFO("Initial pose Y : %.4f " , segment_plan[i].pose.position.y);

            LOG(INFO) << "Initial pose Z : " << segment_plan[i].pose.position.z;
            ROS_INFO("Initial pose Z : %.4f " , segment_plan[i].pose.position.z);
            
            switch_map_inf.request.location.header.frame_id = switch_map_name;
            switch_map_inf.request.location.pose.position = segment_plan[i].pose.position;
            //switch_map_inf.request.location.pose.orientation = robot_pose.pose.orientation; /* 210421 char */
            //printf("map_inf_z %f map_inf_w %f",switch_map_inf.request.location.pose.orientation.z,switch_map_inf.request.location.pose.orientation.w);
                                                         
            ros::Duration(0.5).sleep(); /* 210421 char */

            if (switch_map_client.call(switch_map_inf))
            {
              LOG(INFO) << "Switch map succeed!";
              ROS_INFO("Switch map succeed!");
            }
            else
            {
              LOG(INFO) << "Switch map failed!";
              ROS_INFO("Switch map failed!");
            }
            ros::Duration(2.0).sleep();
            //map_score_limit = temp_limit;
            /* 210421 char add */
            map_switch.data = false;
            map_switch_pub.publish(map_switch);
            /* 210421 char add end */

            //0524 char add 
            while(remathstop && (!pausestop))
            //{
              ROS_WARN("waitting match");
            //  LOG(INFO) << "waitting match";            
            //}
          }
          else
          {
            LOG(INFO) << "The same map not switch";
            ROS_INFO("The same map not switch");
          }
        }
        /* 210409jimc add end */


        /* 210311jimc add start */
        nav_param_select = segment_plan_pathinfo[i].laserMode;
        LOG(INFO)<<"nav_param_select:" << nav_param_select;

        /* 211019jimc if( nav_param_select != nav_param_select_previou ) */
        /* 211019jimc add start */
        if(    ( nav_param_select == nav_param_select_previou )
            || ( ( nav_param_select == 0 ) && (nav_param_select_previou == 1 ) )
          )
        {
          ROS_INFO("The same parameter not change!");
        }
        else
        /* 211019jimc add end */
        {
          //0412 char add
          cmd_vel.linear.x = cmd_vel.linear.y = cmd_vel.angular.z = 0;
          velocity_pub.publish(cmd_vel);
          //0412 char end

          /* 210407jimc add */
          if( nav_param_select == 1 )
          {
            change_nav_param(0);
            switch_flag = 0;
          }
          else
          {
            obsdec_avoid_flg = 0; /* 211008jimc */
            change_nav_param(nav_param_select);
            switch_flag = nav_param_select;
          }
          ROS_INFO("Change parameter Finish!");
          /* 210407jimc add end */

          
          if(nav_param_select == 2)
          {
            map_score_limit = map_score_high;
            switvh_bay_flag = true;
          }
          else
          {
            map_score_limit = map_score_low;
          }
           /* 210407jimc change_nav_param(nav_param_select); */
          nav_param_select_previou = nav_param_select;
        }
        /* 211019jimc mark start
        else
        {
          ROS_INFO("The same parameter not change!");   /* 210413jimc 
        }
        211019jimc mark end */
        /* 210311jimc add end */

        /* 210311jimc while (ros::ok && !(positionon && angleon) && (AlarmCode == 0) && (preemptreq == false)) */
        while (ros::ok() && !(positionon && angleon) && (AlarmCode == 0) && (preemptreq == false) && !emptycancel ) /* 210311jimc  */
        {
          //check is new task preempt requested, or pause stop
          /* 210518jimc if (as->isPreemptRequested() || pausestop == 1 || forwardstop == 1 || remathstop ==1) */
          if (as->isPreemptRequested() || forwardstop == 1 || remathstop ==1) /* 210518jimc */
          {

            if (pausestop == 1 && IsErrorStopDone(linear_vel, RoW, current_cmd_vel))
            {
              printf("pausestop\n");
              LOG_EVERY_N(WARNING, 80) << "OBSTACL PAUSE STOPPING";
            }
            if (forwardstop == 1 && IsErrorStopDone(linear_vel, RoW, current_cmd_vel))
            {
              preemptreq = true;
              LOG(INFO) << "Forward Stop & End Task";
            }

            if (remathstop == 1 && IsErrorStopDone(linear_vel, RoW, current_cmd_vel))
            {
              //preemptreq = true;
              printf("remathstop\n");

              LOG(INFO) << "map score too low & rematch map";
            }
            if (as->isPreemptRequested())
              preemptreq = true;
          }
          //on path tracking
          else
          {
            if( pausestop == 1 )
            {
              printf("pausestop\n");
            }
            planner_costmap_ros_->getRobotPose(grobot_pose);
            robot_ang = tf::getYaw(grobot_pose.getRotation());
            poseStampedTFToMsg(grobot_pose, feedback.base_position);
            as->publishFeedback(feedback);
            plan_pub.publish(cglobal_plan);
            poseStampedMsgToTF(segment_plan[i], nextgoal_pose);

            if (i == 0)lastgoal_pose = grobot_pose;
            else poseStampedMsgToTF(segment_plan[i - 1], lastgoal_pose);

            nextdist_err = diserr(nextgoal_pose, grobot_pose);
            nextangle_err = angerr(nextgoal_pose, grobot_pose);
            finalroangle_err = frangerr(subfinalgoal_pose, grobot_pose);
            finalrodist_err = diserr(subfinalgoal_pose, grobot_pose);
            float line_ang = CalculateLineAngle(lastgoal_pose,nextgoal_pose);

            geometry_msgs::PoseStamped next_pose;

            /* 210421 char if(i == 0) */
            if(i == 0 || segment_plan_pathinfo[i].changeMap==2)   /* 210421 char */
            {
              if( goal_nums != 1 )  /* 210422jimc */
              {
                anglefw = true;
                positionon = true;
              }
              //ROS_ERROR("0426 i =%d ,sub_n-1 =%d",i,subgoal_nums-1);
              if(i != subgoal_nums-1)angleon = true; //0112  if not final point passs angle
              
              /* 210421 char add */
              if( segment_plan_pathinfo[i].changeMap==2)
              {
                ROS_ERROR("change map");

                goal_result =1;
                set_goal=1;
                //angleon = true;
                //lastVisitedNode = next_pose.header.seq;
              }  

              /* 210421 char add end */
            }
            //0322 change
            //if((i != 1 && nextangle_err < 0.5*M_PI/180) || segment_plan_pathinfo[i].direction==1 ) anglefw = true; //0115 test
            /* 210421 char if((i == 0 && nextangle_err < 0.5*M_PI/180) || segment_plan_pathinfo[i].direction==1 ) anglefw = true; //0115 test */
            //i==1 ||  nextangle_err > 0.5*M_PI/180 => anglefw =0
            //0429 chang direction chang base
            //printf("%f i %d",nextangle_err,segment_plan_pathinfo[i>0?i-1:0].direction);

            //0511 if(( fabs(nextangle_err )< 0.1*M_PI/180) || (segment_plan_pathinfo[i>0?i-1:0].direction==1 ))
            /* 210901jimc if(   (i!=0 && ( fabs(nextangle_err )< 0.1*M_PI/180))  */
            if(   ( i!=0 && ( fabs(nextangle_err )< 5*M_PI/180) )   /* 210901jimc */
               || (segment_plan_pathinfo[i>0?i-1:0].direction==1 )
              )
            {
              //ROS_WARN("i=%d s=%d ",i,s); //210922 test
              anglefw = true; /* 210421 char */
              //printf("anglefw\n");
            }
            //0504 pass goal 0520 mark
            //printf("0504 test i %d\n",i);
            /* 210913jimc add */
            if( pass_point_check == true )
            {
              anglefw = true;
              ROS_INFO("[anglefw] pass point no Turning_loop");
            }
            /* 210913jimc add end */
            /* 
            if(fabs(nextdist_err)<0.1 && i==0 && set_goal ==0 && goal_result==0)
            {
              set_goal =1;
              goal_result =1;
              anglefw = true;
              ROS_INFO("too close to goal, pass the goal");
            }
            */
              //0504 pass goal end
            if(!anglefw)
            {
              linear_vel.x = 0;
              linear_vel.y = 0; //20210201wusl
              anglefw = Turning_loop(RoW,nextangle_err);
              //ROS_INFO("[anglefw] Turning_loop");   /* 210826jimc */

              if(anglefw)
              {
                LOG(INFO) << "[barcodemove] anglefw";
                ROS_INFO("[barcodemove] anglefw");
              }
              //ROS_INFO("nextangle_err:%f",nextangle_err);
            }
            else
            {
              next_pose.pose = segment_plan[i].pose;
              next_pose.header.seq = segment_plan[i].header.seq;
              //0323  back to last piont  
              if(obst_back == 1 && segment_plan_pathinfo[i].laserMode ==2 )
              {
                next_pose.pose = segment_plan[i-1].pose;
                next_pose.header.seq = segment_plan[i-1].header.seq;
                //back_flag =0;
                s=segno-1;
                i=subgoal_nums-1;
                back_flag =1;
              }
              //0323  back to last piont  end

             
              //0323 set goal ============================================================================================================
              /* 210907jimc if(!set_goal && !positionon && !check_tag) //publish goal to slam */
              //publish goal to slam
              if(!set_goal && !positionon ) /* 210907jimc */
              {
                LOG(INFO) << "nextangle_err:" << nextangle_err*180/M_PI ;
                ROS_INFO("nextangle_err:%f",nextangle_err*180/M_PI);
                LOG(INFO) << "line_ang:" << line_ang*180/M_PI;
                ROS_INFO("line_ang:%f",line_ang*180/M_PI);
                //printf("subgoal_nums=%d",subgoal_nums);//0323
                // 0115 far target test start
                #if 0 /* 210824jimc mark */
                if(i == subgoal_nums-1 && search_tag && nextID < 100) //search_tag=0 turn off function
                {
                  printf("0324 chek do something\n");

                  LOG(INFO) << "[SLAM] orig x:%f y:" << next_pose.pose.position.x,next_pose.pose.position.y;
                  ROS_INFO("[SLAM] orig x:%f y:%f",next_pose.pose.position.x,next_pose.pose.position.y);
                  LOG(INFO) << "[SLAM] set far target";
                  ROS_INFO("[SLAM] set far target");
                  double x_dis = nextgoal_pose.getOrigin().x() - lastgoal_pose.getOrigin().x();
                  double y_dis = nextgoal_pose.getOrigin().y() - lastgoal_pose.getOrigin().y();
                  LOG(INFO) << "[SLAM] x_dis:%f y_dis:" << x_dis,y_dis ;
                  ROS_INFO("[SLAM] x_dis:%f y_dis:%f",x_dis,y_dis);
                  if(fabs(x_dis) > fabs(y_dis)) next_pose.pose.position.x += Sign(x_dis) * 0.1;
                  else next_pose.pose.position.y += Sign(y_dis) * 0.1;

                  printf("0324 chek 2 end\n");
                }
                #endif  /* 210824jimc mark end */
                // 0115 far target test end

                //want traceang  next position 20210204 char add
                //line_ang=tf::getYaw(nextgoal_pose.getRotation());
                //add end
                //next_pose.pose.orientation.z=nextgoal_pose.getRotation().getZ();
                //next_pose.pose.orientation.z=nextgoal_pose.getRotation().getW();
                
                //0316
                printf("direction %d\n",segment_plan_pathinfo[i].direction);
                
                //0324 translate to backward line_ang
                //printf("0406 1st line_ang %f\n",line_ang*180/M_PI);
                if((segment_plan_pathinfo[i-1<0?0:i-1].direction))//not roration
                {
                  //float delt_ang =fabs((robot_ang< 0 ?(2*M_PI-robot_ang):robot_ang) -(line_ang< 0 ?(2*M_PI-line_ang):line_ang)); 
                  //if (delt_ang >M_PI)delt_ang -=M_PI;
                  //if(delt_ang > (M_PI/2))
                  //if(fabs(line_ang) >= (M_PI/2))//backward

                  //0429 char change line_ang calculate base point 
                  //LOG(INFO) << "[0429 ang test]before change line_ang %f" <<line_ang*180/M_PI ;
                  //line_ang = CalculateLineAngle(grobot_pose,nextgoal_pose);
                  //printf("robot_ang %f ",robot_ang*180/M_PI);
                  //LOG(INFO) << "[0429 ang test]robot_ang %f" <<robot_ang*180/M_PI ;
                  //LOG(INFO) << "[0429 ang test]after change line_ang %f" <<line_ang*180/M_PI ;
                  float delt_ang =fabs(robot_ang -line_ang);
                  if(delt_ang > M_PI)
                  {
                    delt_ang = fabs(2*M_PI -delt_ang);
                  }
                  if(delt_ang > (M_PI/2))
                  {  
                    printf("rob_ang %f line %f  delt %f \n",robot_ang,line_ang,delt_ang );
                    line_ang = line_ang - Sign(line_ang)*M_PI;
                    back_flag =1;
                    printf("chang dirt");
                  }
                }
                //0316 end
                printf("line_ang %f\n",line_ang*180/M_PI);
                //if(segment_plan_pathinfo[i].laserMode ==2)
                next_pose.pose.orientation.z = sin(line_ang/2);
	              next_pose.pose.orientation.w = cos(line_ang/2);
                

                //20210204 end
                move_base_msgs::MoveBaseGoal movebase_goal;

                movebase_goal.target_pose.header.seq = next_pose.header.seq;
                movebase_goal.target_pose.header.frame_id = "map";
                movebase_goal.target_pose.header.stamp = ros::Time::now();
 	              movebase_goal.target_pose.pose=next_pose.pose;

                LOG(INFO) << "goal publish";
                LOG(INFO) << "i=" << i << " move to " << movebase_goal.target_pose.header.seq ;
                LOG(INFO) << "x:" << movebase_goal.target_pose.pose.position.x <<  "y:" << movebase_goal.target_pose.pose.position.y;
                LOG(INFO) << "z:" << movebase_goal.target_pose.pose.orientation.z <<  "w:" << movebase_goal.target_pose.pose.orientation.w;

                ROS_INFO("goal publish");
                ROS_INFO("i=%d move to %d",i,movebase_goal.target_pose.header.seq);
                ROS_INFO("x:%f y:%f"
                         ,movebase_goal.target_pose.pose.position.x
                         ,movebase_goal.target_pose.pose.position.y);
                ROS_INFO("z:%f w:%f"
                         ,movebase_goal.target_pose.pose.orientation.z
                         ,movebase_goal.target_pose.pose.orientation.w);
                replan_goal = movebase_goal;//20210303 char
                ac.sendGoal(movebase_goal);
                tempdist_err=nextdist_err;
	              set_goal = 1;
                misstion_exe_flg = 1;             /* 210401jimc */
                pass_point_check = false;         /* 210913jimc */
                velctrl_flag =0;  /*0929 char*/
	            } //set_goal ============================================================================================================
              
              //replan ============================================================================================================
              //0323add change move mode
              if((replan_goal_flag == 1) && (segment_plan_pathinfo[i].laserMode !=2) )
              {
                //0323
                //change_nav_param(segment_plan_pathinfo[i].laserMode);
                //0323
                replan_goal_flag =0;
                misstion_exe_flg = 1;             /* 210407jimc */
                //anglefw=0;
                ac.sendGoal(replan_goal);
              }
              //0323 chang move mode end

              //0323add back to last point
              /* 210401jimc mark start
              if(replan_goal_flag == 1 &&segment_plan_pathinfo[i].laserMode ==2 )
              {
                replan_goal_flag = 0; 
                obst_back =1;
                
                //anglefw=0;
                set_goal=0;
                //ac.sendGoal(replan_goal);
              }
              210401jimc mark end */
              //0323 back to last point end
              //repaln end ============================================================================================================
              //slam goal reached============================================================================================================
              //0428 test pass final angerr
              //if(nextdist_err < 0.1)
              //{
                //goal_result =1;
                //ac.cancelAllGoals();//0322
              //}
              if(goal_result) //slam goal reached
              {
                goal_result =0;
                bay_flag = false;     /* 210511GU */
                bay_request.data = 0; /* 210511GU */
                bay_alignment_pub.publish(bay_request); /* 210511GU */
                //ROS_ERROR("0426 i =%d ,sub_n =%d",i,subgoal_nums-1);
                //ROS_ERROR("0426 s =%d ,segno =%d",s,segno-1);
                if(s!=segno-1 || i!=subgoal_nums-1)
                {
                  angleon = true;
                }
                //printf("0426 %d\n",angleon);
                /* 210421 char if(nextdist_err < 0.1)  */
                if(nextdist_err < 0.1 || segment_plan_pathinfo[i].changeMap==2)   /* 210421 char */
                {
                  positionon = true;
                  lastVisitedNode = next_pose.header.seq;
                  LOG(INFO) << "[SLAM] Endpoint goal reached";
                  ROS_INFO("[SLAM] Endpoint goal reached");
                }
                else   //0317 
                {
                  #if 0 /* 210824jimc */
                    /* 210824jimc float fix_vx,fix_vy;  */
                    double fix_vx,fix_vy; /* 210824jimc */
                    double disX_err = subfinalgoal_pose.getOrigin().x() - grobot_pose.getOrigin().x();
                    double disY_err = subfinalgoal_pose.getOrigin().y() - grobot_pose.getOrigin().y();
                    /* 210824jimc positionon = fix_diserr(disX_err,disY_err,fix_vx,fix_vy); */
                    positionon = fix_diserr(fix_vx,fix_vy,disX_err,disY_err); /* 210824jimc */
                    /* 210824jimc linear_vel.x =fix_vx;
                    linear_vel.y =fix_vy; */
                    ROS_INFO("fix_diserr: %f,%f",fix_vx,fix_vy);  
                             
                    if(positionon)
                    {
                      LOG(INFO) << "[barcodemove] positionon";
                      ROS_INFO("[barcodemove] positionon");
                    }
                  #endif  /* 210824jimc */  
                  
                  /* 210913jimc add end */
                  //positionon = true;  /* 210824jimc */
                  //lastVisitedNode = next_pose.header.seq;  /* 210824jimc */
                  /* 210913jimc add */
                  if( pass_point_check == true )
                  {
                    positionon = true;
                    lastVisitedNode = next_pose.header.seq;
                    ROS_INFO("[SLAM] Endpoint goal reached by pass point!!");
                  }
                  else
                  {
                    ROS_INFO("[SLAM] Endpoint goal reached but not in position!!");   /* 210909jimc */
                  }
                }
                /* 210824jimc mark 
                if(s == segno-1 && i == subgoal_nums-1 && !gettag && nextID < 100)
                {
                  LOG(INFO) << "[SLAM] Missing tag on Endpoint";
                  ROS_INFO("[SLAM] Missing tag on Endpoint");
                  AlarmCode = 7;
                }
                210824jimc mark */
                set_goal = 0;
                misstion_exe_flg = 0;             /* 210401jimc */
                LOG(INFO) << "[SLAM] goal_result anglefw:%d angleon:%d positionon:" << anglefw,angleon,positionon ;
                ROS_INFO("[SLAM] goal_result anglefw:%d angleon:%d positionon:%d",anglefw,angleon,positionon);
                
                ROS_INFO("get reached distance:%f",nextdist_err);           /* 210910jimc */
                ROS_INFO("get reached angle:%f",nextangle_err*180/M_PI); /* 210910jimc */
              } //goal_result
              //slam goal reached end ============================================================================================================
              
              //pass point============================================================================================================
              //if(i != 0 && i != subgoal_nums - 1 && nextdist_err < 0.3) //pass point
              /* 210825jimc if(!corner_flag) //0427 */

              if(!segment_plan_pathinfo[i].corner)  /* 210901jimc */
              {
                /* 210901jimc if((s!=segno-1) && nextdist_err < 0.3) //pass point   //0310 char change */
                if(    ( ( i != 0 ) && ( s != segno-1 ) )                      /* 210901jimc */
                    && ( nextdist_err < 0.7)                  /* 210901jimc */
                  ) 
                {
                  positionon = true;
                  angleon = true;
                  /* 210901jimc ac.cancelAllGoals();//0322 */
                  pass_point_check = true;  /* 210907jimc */
                  //set_goal =0; //0511 test
                  lastVisitedNode = next_pose.header.seq;
                  /* 210825jimc add */
                  ROS_INFO("pass target nextgoal_pose x:%f y:%f"
                         ,nextgoal_pose.getOrigin().x()
                         ,nextgoal_pose.getOrigin().y());

                  ROS_INFO("pass target grobot_pose x:%f y:%f"
                         ,grobot_pose.getOrigin().x()
                         ,grobot_pose.getOrigin().y());
                  /* 210825jimc add end */        
                  
                  LOG(INFO) << "pass target distance:",nextdist_err ;
                  ROS_ERROR("pass target distance:%f",nextdist_err);
                }
              }

              //pass point============================================================================================================
              //20210111wusl add start============================================================================================================
              //ROS_ERROR("i= %d subgoal_nums =%d s =%d segmo =%d ",i,subgoal_nums,s,segno);
              //if(i==subgoal_nums-1 && !positionon) //0113 remove s=segno-1
              if(i!=subgoal_nums && !positionon) //0412 char chang 
              {
		            //20210111wusl add start
                //if(slam_cmd_vel.linear.x==0)//20210203wusl
                if(slam_cmd_vel.linear.x==0 && slam_cmd_vel.linear.y==0) //20210203wusl
                {
                  iSmooth_Step=99;
                  bInSmooth=false;
                  dSmooth_time_start = ros::Time::now().toSec();
                }

                if(goalRange5 !=0)
                {
                  if(nextdist_err > (goalRange5*2))
                  iSmooth_Step=99;
                }
                else if(goalRange4 !=0)
                {
                  if(nextdist_err > (goalRange4*2))
                  iSmooth_Step=99;
                }
                else if(goalRange3 !=0)
                {
                  if(nextdist_err > (goalRange3*2))
                  iSmooth_Step=99;
                }
                else if(goalRange2 !=0)
                {
                  if(nextdist_err > (goalRange2*2))
                  iSmooth_Step=99;
                }
                else if(goalRange1 !=0)
                {
                  if(nextdist_err > (goalRange1*2))
                  iSmooth_Step=99;
                }
                else
                {
                  iSmooth_Step=99;
                }

                if(bInSmooth)
                {
                  if(fabs(ros::Time::now().toSec() - dSmooth_time_start)>15)
                  {
                    iSmooth_Step=99;
                    bInSmooth=false;
                    dSmooth_time_start = ros::Time::now().toSec();
                  }
                }
                //20210111wusl add end============================================================================================================
                //0330 check path error
                /*if(nextdist_err>tempdist_err)
                {
                  printf("nexdist_err over");
                  slam_cmd_vel.angular.z=0.0;
                  slam_cmd_vel.linear.x=0.0;
                  slam_cmd_vel.linear.y=0.0;
                }
                */
                //0324 when near point can fix position========================================================= 
                if(segment_plan_pathinfo[i].laserMode!=2)
                {
                  //printf("nextdist_err %f\n",nextdist_err);
                  //0427 navigation move limit
                  //if(segment_plan_pathinfo[i].laserMode!=2)
                  //if( (fabs(nextdist_err) > 0.7) && (switch_flag == 0) )                
                  {  
                    //printf("0412 y_limit test\n");
                    //float drift_y = nextdist_err *cos(CalculateLineAngle(nextgoal_pose, grobot_pose));
                    //ROS_ERROR("0427 test drift_y %f,dist_err %f ang_err %f",drift_y,nextdist_err,CalculateLineAngle(nextgoal_pose, grobot_pose));
                    //if(fabs(drift_y) > 0.1)
                      //slam_cmd_vel.linear.y=(drift_y>0?-1:1)*0.1;
                    //else
                    {
                      //slam_cmd_vel.linear.y=0.0;
                    }
                  }
                    //slam_cmd_vel.linear.y=0.0;
                  //0428 
                  //if(switch_flag !=0 || segment_plan_pathinfo[i-1<0?0:i-1].direction!=0 )  
                  if(switch_flag ==2 || segment_plan_pathinfo[i-1<0?0:i-1].direction!=0 || segment_plan_pathinfo[i].direction!=0 )  //1019 char change
                    slam_cmd_vel.angular.z=0.0;

                }
                //0324=========================================================================================
                //0324 in bay fix angle ============================================================================================================
                #if 0   /* 210825jimc mark*/
                if(segment_plan_pathinfo[i].laserMode == 2)
                {   
                    //slam_cmd_vel.linear.y =0.0;
                    //slam_cmd_vel.angular.z =0.0;
                    nextangle_err = angerr(nextgoal_pose, grobot_pose);
                    //0503
                    
                    if(!back_flag)
                    { 
                      if(fabs(nextangle_err )> 0.1*M_PI/180  )
                      {    
                        //printf("forward ang %f\n",nextangle_err);
                        //0504
                        //0511slam_cmd_vel.angular.z =nextangle_err/2;
                        //printf("angular %f\n",nextangle_err);
                        //Turning_loop(RoW,nextangle_err);
                        //0323 test
                      }
                    }
                    else
                    {
                      if(fabs(nextangle_err ) > M_PI/2)
                      {
                        if(fabs(180*M_PI/180-fabs(nextangle_err )) > 0.1*M_PI/180  )
                        {    
                          //0323 test
                          //printf("2 %f\n",(180*M_PI/180-fabs(nextangle_err )));
                          //printf("backward  ang %f (>90)\n",(180*M_PI/180-fabs(nextangle_err )));
                          //0504
                          //slam_cmd_vel.angular.z = Sign((-(180*M_PI/180-fabs(nextangle_err ))*Sign(nextangle_err))) * 0.007;
                          //0511slam_cmd_vel.angular.z= (-(180*M_PI/180-fabs(nextangle_err ))*Sign(nextangle_err))/2;
                          
                          //Turning_loop(RoW,(-(180*M_PI/180-fabs(nextangle_err ))*Sign(nextangle_err)));
                          
                        }
                      }
                      else
                      {
                        
                        if(fabs(nextangle_err ) > 0.1*M_PI/180  )
                        {    
                          //printf("backward ang %f(<90)\n",(fabs(nextangle_err )));
                          //0504
                          //Turning_loop(RoW,nextangle_err );
                          //0510
                          //slam_cmd_vel.angular.z = Sign(nextangle_err) * 0.007;
                          //0511slam_cmd_vel.angular.z = (nextangle_err)/2;
 
                        //0323 test
                        }
                      }
                    }
                }
                #endif    /* 210825jimc mark */
                //0324 fix angle end============================================================================================================
                // limt_vel ============================================================================================================
                if((nextdist_err < goalRange1 && iSmooth_Step >1) || iSmooth_Step==1) //20210111wusl
                //if(nextdist_err < goalRange1)                                       //20210111wusl
                {
                  fixed_cmd_vel.linear.x = slam_cmd_vel.linear.x * goalSpeed1;      //0114 test
                  fixed_cmd_vel.linear.y = slam_cmd_vel.linear.y * goalSpeed1;      //20210203wusl
                  fixed_cmd_vel.angular.z = slam_cmd_vel.angular.z * goalSpeed1;    //0114 test
                  //20210111wusl add start
                  iSmooth_Step=1;
                  if( !bInSmooth )
                  {
                    bInSmooth=true;
                    dSmooth_time_start = ros::Time::now().toSec();
                  }
                  //20210111wusl add end
                }
                else if((nextdist_err < goalRange2 && iSmooth_Step>2) || iSmooth_Step==2) //20210111wusl
                //else if(nextdist_err < goalRange2)                                      //20210111wusl
                {
                  fixed_cmd_vel.linear.x = slam_cmd_vel.linear.x * goalSpeed2;
                  fixed_cmd_vel.linear.y = slam_cmd_vel.linear.y * goalSpeed2;//20210203wusl
                  fixed_cmd_vel.angular.z = slam_cmd_vel.angular.z * goalSpeed2;
                  //20210111wusl add start
                  iSmooth_Step=2;
                  if( !bInSmooth )
                  {
                    bInSmooth=true;
                    dSmooth_time_start = ros::Time::now().toSec();
                  }
                  //20210111wusl add end
                }
                else if((nextdist_err < goalRange3 && iSmooth_Step>3) || iSmooth_Step==3) //20210111wusl
                //else if(nextdist_err < goalRange3)                                     //20210111wusl
                {
                  fixed_cmd_vel.linear.x = slam_cmd_vel.linear.x * goalSpeed3;
                  fixed_cmd_vel.linear.y = slam_cmd_vel.linear.y * goalSpeed3;//20210203wusl
                  fixed_cmd_vel.angular.z = slam_cmd_vel.angular.z * goalSpeed3;
                  //20210111wusl add start
                  iSmooth_Step=3;
                  if( !bInSmooth )
                  {
                    bInSmooth=true;
                    dSmooth_time_start = ros::Time::now().toSec();
                  }
                  //20210111wusl add end
                }
                else if((nextdist_err < goalRange4 && iSmooth_Step>4) || iSmooth_Step==4) //20210111wusl
                //else if(nextdist_err < goalRange4)                                     //20210111wusl
                {
                  fixed_cmd_vel.linear.x = slam_cmd_vel.linear.x * goalSpeed4;
                  fixed_cmd_vel.linear.y = slam_cmd_vel.linear.y * goalSpeed4;//20210203wusl
                  fixed_cmd_vel.angular.z = slam_cmd_vel.angular.z * goalSpeed4;
                  //20210111wusl add start
                  iSmooth_Step=4;
                  if( !bInSmooth )
                  {
                    bInSmooth=true;
                    dSmooth_time_start = ros::Time::now().toSec();
                  }
                  //20210111wusl add end
                }
                else if((nextdist_err < goalRange5 && iSmooth_Step>5) || iSmooth_Step==5) //20210111wusl
                //else if(nextdist_err < goalRange5)                                     //20210111wusl
                {
                  fixed_cmd_vel.linear.x = slam_cmd_vel.linear.x * goalSpeed5;
                  fixed_cmd_vel.linear.y = slam_cmd_vel.linear.y * goalSpeed5;//20210203wusl
                  fixed_cmd_vel.angular.z = slam_cmd_vel.angular.z * goalSpeed5;
                  //20210111wusl add start
                  iSmooth_Step=5;
                  if( !bInSmooth )
                  {
                    bInSmooth=true;
                    dSmooth_time_start = ros::Time::now().toSec();
                  }
                  //20210111wusl add end
                }
                else
                {
                  fixed_cmd_vel = slam_cmd_vel;
                }

                /* 210511GU add */
                if(nav_param_select==2 && bay_flag==false && set_goal && switvh_bay_flag)    //2021GU
                {
                  if( vel_cancel_pub == false )
                  {
                      bay_request.data = 1;
                      bay_alignment_pub.publish(bay_request);
                      switvh_bay_flag =false;
                  }
                  vel_cancel_pub = true;
                }
                else
                {
                  vel_cancel_pub = false;
                  bay_flag = true;
                  //switvh_bay_flag =false;
                  //printf("bay_flag=%d\n",bay_flag);
                }
                /* 210511GU add end */

                //ROS_ERROR("laser_left %f laser_right %f",laser_left,laser_right);
                if(    (nav_param_select == 2) 
                    && get_slam_cmd 
                    && (bay_flag == true)   /* 210511GU */
                    && (pausestop == false)   /* 210511GU */
                  )             //20210330GU
                {
                  ROS_INFO("Alignment center");
                  double bay_offset = 0.0;
                  if(laser_right==-1 && laser_left!=-1)
                      bay_offset = laser_left;
                  else if(laser_right!=-1 && laser_left==-1)
                      bay_offset = laser_right;
                  else if(laser_right!=-1 && laser_left!=-1)
                      bay_offset = (laser_left + laser_right)/2 - laser_right;
                  //LOG(INFO) << "laser_left:"<<laser_left ;
                  //LOG(INFO) << "laser_right:"<<laser_right ;
                  //ROS_INFO("bay_offset:%.3f",bay_offset);
                  //LOG(INFO) << "bay_offset:"<< bay_offset ;
                  ROS_INFO("left %f right %f",laser_left,laser_right);

                  //if(fabs(offset) > 0.03 && fabs(laser_left - laser_right) <= 0.25)
                  if(fabs(bay_offset)>0.01  && laser_left!=-1 && laser_right!=-1)  
                  {
                      //0510
                      /* 211013jimc fixed_cmd_vel.linear.y = bay_offset * 0.15; */
                      fixed_cmd_vel.linear.y = bay_offset ; /* 211013jimc */
                      //fixed_cmd_vel.linear.y =bay_offset*fabs(fixed_cmd_vel.linear.x);
                      //fixed_cmd_vel.linear.y =offset*fabs(fixed_cmd_vel.linear.x)/1.414;
                      
                      //fixed_cmd_vel.linear.y = offset * 0.1;
                  }//0503
                  else if(laser_left==-1 && laser_right!=-1 && (bay_offset>0.46 || bay_offset<0.44))
                  {
                      if(bay_offset >= 0.48)
                        fixed_cmd_vel.linear.y = -0.01;
                      else if(bay_offset <= 0.35)
                        fixed_cmd_vel.linear.y = 0.01;
                      else
                        fixed_cmd_vel.linear.y = 0.0; 
                  } 
                  else if(laser_left!=-1 && laser_right==-1 && (bay_offset>0.46 || bay_offset<0.44))   
                  {
                      if(bay_offset >= 0.48)
                        fixed_cmd_vel.linear.y = 0.01;
                      else if(bay_offset <= 0.35)
                        fixed_cmd_vel.linear.y = -0.01;
                      else 
                        fixed_cmd_vel.linear.y = 0.0;
                  }
                  else
                  {
                    ROS_ERROR("laser_left %f laser_right %f",laser_left,laser_right); /* 210830jimc */
                    if(fabs(laser_left - laser_right)>= 2)  /* 210830jimc */
                    {
                      fixed_cmd_vel.linear.y = 0.0;
                    }
                  }
                  LOG(INFO) << "bay_offset_y:"<<fixed_cmd_vel.linear.y ;
                  ROS_INFO("bay_offset_y:%.3f",fixed_cmd_vel.linear.y);
                  //fixed_cmd_vel.angular.z = 0.0;  //0503
                }
                else if(nav_param_select == 2)
                {
                  fixed_cmd_vel.linear.y = 0.0;
                }

                //ROS_INFO("[barcodemove] nextdist_err:%f Step%d",nextdist_err,iSmooth_Step);
                slam_cmd_vel = fixed_cmd_vel;

              }
              //limt_vel end============================================================================================================

              //20210111wusl add end

              //final point fix angle====================================================================================================
              //ROS_ERROR("s=%d segno-1=%d i=%d subgoal_nums-1 =%d positionon=%d",s,segno-1,i,subgoal_nums-1,positionon);
              if(s==segno-1 && i==subgoal_nums-1 && positionon)
              {
                linear_vel.x = 0;//20210110wusl
                linear_vel.y = 0;//20210203wusl

                //finalroangle_err = tf::getYaw(subfinalgoal_pose.getRotation()) - tagang; //0119 turn to target depend tagang
                finalroangle_err = tf::getYaw(subfinalgoal_pose.getRotation()) - tf::getYaw(grobot_pose.getRotation()); //not use bartag but artag need to check 0209
                //ROS_ERROR("z:%f x:%f ",subfinalgoal_pose.getRotation().getZ(),subfinalgoal_pose.getRotation().getW());
                //ROS_ERROR("z:%f x:%f ",grobot_pose.getRotation().getZ(),grobot_pose.getRotation().getW());
                
                finalroangle_err = atan2(sin(finalroangle_err), cos(finalroangle_err));  //0119 turn to target depend tagang
                //0426 char change
                if(segment_plan_pathinfo[i].direction==0)
                {
                //if(segment_plan_pathinfo[i].laserMode!=2)
                
                  angleon = Turning_loop(RoW,finalroangle_err);
                  //ROS_INFO("[angleon] %d, finalroangle_err = %f",angleon, finalroangle_err);   /* 210826jimc */
                }
                else
                {
                  angleon =1;
                }

                if(angleon)
                {
                  LOG(INFO) << "[barcodemove] angleon";
                  ROS_INFO("[barcodemove] angleon");
                  /* 210824jimc positionon =false; //0316 */
                  //bool data =true;
                  //map_update_pub.publish(data);
                  //ros::Duration(1.0).sleep();//0330
                }                
                //ROS_INFO("finalroangle_err:%f",finalroangle_err);
              }
              //final point fix angle end===============================================================================================
              /*********/
              /*need updata map match*/
              /*********/
              //final point refix point end================================================================================================
              if(s==segno-1 && i==subgoal_nums-1 && angleon)
              {
                
                RoW =0.0;
                #if 0 /* 210824jimc */
                /* 210824jimc float fix_vx,fix_vy;  */
                double fix_vx,fix_vy; /* 210824jimc */
                double disX_err = subfinalgoal_pose.getOrigin().x() - grobot_pose.getOrigin().x();
                double disY_err = subfinalgoal_pose.getOrigin().y() - grobot_pose.getOrigin().y();
                /* 210824jimc positionon = fix_diserr(disX_err,disY_err,fix_vx,fix_vy); */
                positionon = fix_diserr(fix_vx,fix_vy,disX_err,disY_err); /* 210824jimc */
                linear_vel.x =fix_vx;
                linear_vel.y =fix_vy;
                printf("0323 test\n");              
                if(positionon)
                {
                  LOG(INFO) << "[barcodemove] positionon";
                  ROS_INFO("[barcodemove] positionon");
                  lastVisitedNode = next_pose.header.seq;
                }
                #endif  /* 210824jimc */
                //positionon = true;  /* 210824jimc */
                //lastVisitedNode = next_pose.header.seq;  /* 210824jimc */

                // /ROS_INFO("disX_err:%f ,disY_err:%f ",disX_err,disY_err);
                //0323
                if(obst_back)
                {
                  obst_back = 0;
                  //AlarmCode =91; //back to  last point 
                }
              } 
              //final point refix point end============================================================================================            
            }
            //0924 char test
            //#if 0
            //line_ang = CalculateLineAngle(grobot_pose,nextgoal_pose); 
            line_ang =  atan2((TEB_path_pose.position.y - grobot_pose.getOrigin().y()), (TEB_path_pose.position.x - grobot_pose.getOrigin().x()));//1104 char  /* teb_path */
            double fix_ang = tf::getYaw(grobot_pose.getRotation())-line_ang;
            
            if(fabs(fix_ang)> 1*M_PI/180 &&fabs(slam_cmd_vel.linear.x)>0.025  && switch_flag == 0 && !pausestop   )
            {
              printf("line %f robot %f fix_ang %f\n",line_ang,tf::getYaw(grobot_pose.getRotation()),sin(fix_ang));
              slam_cmd_vel.angular.z -= 0.7*sin(fix_ang);
            }
            //#endif
            //0924 char test end
          }//on path tracking
          //ros::spinOnce();
          //get slam vel=====================================================================================
          if(set_goal && get_slam_cmd)
          {
            linear_vel.x = slam_cmd_vel.linear.x; 
            linear_vel.y = slam_cmd_vel.linear.y; //20210203wusl
            //if(segment_plan_pathinfo[i].laserMode==2)  //0317
            RoW =  slam_cmd_vel.angular.z;
            //printf("test1\n");
            get_slam_cmd = 0;
          }
          //get slam vel end====================================================================================

          SpeedAdjuster(linear_vel, RoW, 0);
        
          /* 210518jimc add 
          if( isSlowDown == true )
          {
            if(nav_param_select == 2)
            { 
              if( fabs(linear_vel.x) > 0.1)
              {
                linear_vel.x *= 0.8;
              }
              if( fabs(linear_vel.y) > 0.1)
              {
                linear_vel.y *= 0.8;
              }
              RoW *= 0.8;
            }
            else
            {
              if( fabs(linear_vel.x) > 0.1)
              {
                linear_vel.x *= 0.5;
              }
              if( fabs(linear_vel.y) > 0.1)
              {
                linear_vel.y *= 0.5;
              }
              RoW *= 0.5;
            }
          }
           210518jimc add end */
          
          /* 210910jimc add */
          
          if( isSlowDown == true )
          {
            
            if( fabs(linear_vel.x) > 0.1)
            {
              
              linear_vel.x = slowdown_vel*Sign(linear_vel.x);
            }
            if( ( fabs(linear_vel.y) > 0.1 ) && ( nav_param_select != 2 ) ) 
            {
              //linear_vel.x = Sign(linear_vel.x)*0.1/Sign(linear_vel.y);
              linear_vel.y = slowdown_vel*Sign(linear_vel.y);
              
            }
            if( fabs(RoW) > 0.1)
            {
              /* 211014jimc RoW = Sign(RoW)* slowdown_vel/linear_vel.x; */
              RoW = slowdown_vel*Sign(RoW);  /* 211014jimc */
            }
            //ROS_WARN("isSlowDown x %f y %f ",linear_vel.x,linear_vel.y);

          }
          /*
          if( isSlowDown == true )
          {
            if( fabs(linear_vel.x) > 0.1 || fabs(linear_vel.y) > 0.1 ||fabs(RoW) > 0.1)
            {
              float maxspeed[] = {linear_vel.x,linear_vel.y,RoW};
              std::sort(maxspeed, maxspeed+3);
              printf("%lf %lf %lf \n",maxspeed[1],maxspeed[2],maxspeed[3]);
              maxspeed[2] = 0.1/maxspeed[2];
              linear_vel.x *= maxspeed[2];
              linear_vel.y *= maxspeed[2];
              RoW *= maxspeed[2];
              printf("%f  \n",maxspeed[2]);
            }
            ROS_WARN("isSlowDown");
          }
          */
          /* 210910jimc add end */
        

          // if alarm is occured ,publish speed 0 to stop agv
          cmd_vel.linear.x = AlarmCode != 0 ? 0 : linear_vel.x; //speed
          cmd_vel.linear.y = AlarmCode != 0 ? 0 : linear_vel.y; //speed //20210203wusl
          cmd_vel.angular.z = AlarmCode != 0 ? 0 : RoW;         //angle
          if(velctrl_flag ==1 && (switch_flag==1)) //0927 char velctrl
          {
            /*if(!velctrl_flag_count && (fabs(cmd_vel.linear.y)>0.1)){
              velctrl_flag_first_vel = cmd_vel.linear.y;
              velctrl_flag_count++;
              printf("vel_flag = %d   ,",velctrl_flag_count);
            }*/
            printf("linear.y = %f\n",cmd_vel.linear.y);
            cmd_vel.linear.x  =0; //speed
            cmd_vel.linear.y  =slam_cmd_vel.linear.y==0?0:(Sign(slam_cmd_vel.linear.y)*0.10001);//*=1;//=(velctrl_flag_count ? velctrl_flag_first_vel : cmd_vel.linear.y); //speed //20210203wusl
            cmd_vel.angular.z =0; 
            printf("velctrl_flag %d\n",velctrl_flag);
          }
          else if(velctrl_flag ==2 && (switch_flag==1))
          {
            cmd_vel.linear.x  *=1; //speed
            cmd_vel.linear.y  =0;//=(velctrl_flag_count ? velctrl_flag_first_vel : cmd_vel.linear.y); //speed //20210203wusl
            cmd_vel.angular.z =0; 
            printf("velctrl_flag %d\n",velctrl_flag);
          }
          else
          {
            velctrl_flag_count = 0;
            velctrl_flag_first_vel = 0;
          }

          /* 210825jimc if(vel_cancel_pub == false)   /* 210511GU */
          if( vel_cancel_pub == true ) /* 210825jimc */
          {
            /* 210825jimc add */
            cmd_vel.linear.x = 0;
            cmd_vel.linear.y = 0;
            cmd_vel.angular.z = 0;
            ROS_INFO("Bay alignment stop!!");
            /* 210825jimc add end */
          }
          velocity_pub.publish(cmd_vel);
          //ROS_ERROR("pausestop %d",pausestop);

          /* 210825jimc }*/
          r.sleep();
        } // while track loop
        ROS_INFO("While End!!!\n");
      }   // subid loop
          // usleep(50000);
    }     // segment loop
    //keep last state
    lastdynflag = dynflag;
    printf("0406 test %d\n",finalgoalid);
    lastfinalgoalid = finalgoalid;
    last_first_x = firstPoint.x;
    last_first_y = firstPoint.y;
  }
  //check a new task within tag in docking type
  else if ((tracetype == 1 || tracetype == 2) && goal_nums >= 2)
  {
  	//////////////////////////////////////  //20210203wusl need to be modifie by ARTAG function
  	/////  PUT AR TAG Code Here !! ///////  //20210203wusl need to be modifie by ARTAG function 
  	//////////////////////////////////////  //20210203wusl need to be modifie by ARTAG function
  	/* 210205jimc add start */
    AlarmCode = 0;
    system("roslaunch ar_track_alvar mwr_ar_track.launch &");
    LOG(INFO) << "Execute AR Tag task";
    ROS_INFO("Execute AR Tag task");

    int ar_tag_currentID = cglobal_plan.poses[0].header.seq;
    int ar_tag_targetID = cglobal_plan.poses[1].header.seq;

    LOG(INFO) << "AR Tag: currentID: " << ar_tag_currentID << ",goalID: " << ar_tag_targetID ;
    ROS_INFO("AR Tag: currentID: %d ,goalID: %d ",ar_tag_currentID,ar_tag_targetID);

    ar_tag_func_flag.data = 2; //Reset
    artag_func_flag_pub.publish(ar_tag_func_flag);
    
    ros::Duration(1.0).sleep();

    ar_tag_func_flag.data = 0; //Noraml
    artag_func_flag_pub.publish(ar_tag_func_flag);

    ar_tag_sendTagID.current = ar_tag_currentID;  //ar_tag_currentID
    ar_tag_sendTagID.goal = ar_tag_targetID;      //ar_tag_targetID
    artag_tagID_pub.publish(ar_tag_sendTagID);

    ar_tag_move_flag.data = true;
    artag_move_flag_pub.publish(ar_tag_move_flag);

    
    /* 210423jimc while ( ros::ok && (preemptreq == false) && (AlarmCode == 0) && ( ( artagState == 1 ) || (artagState == 2) ) ) */
    while ( ros::ok() && (AlarmCode == 0) && !emptycancel && !pausestop ) /* 210423jimc */
    {
      artagexe = 1;                   /* 210423jimc */
      ROS_INFO("AR Tag ing");         /* 210422jimc */
      
      if(    ( artagState == 3 ) 
          || ( artagState == 4 )
          || ( as->isPreemptRequested() )   /* 210511jimc */
        )
      {
         ROS_INFO("AR Tag interupt");         /* 210422jimc */
          break;
      }

      if( gettag )
      {
        gettag = false;
        lastVisitedNode = ar_tag_targetID;
        LOG(INFO) << "AR Tag in goal";
        ROS_INFO("AR Tag in goal");
        break;
      }
      r1.sleep();       /* 210429jimc */
      ros::spinOnce();  /* 210429jimc */
    }

    /* 210423jimc add start */
    /* Reset ARtag process */
    artagexe = 0;                 /* 210427jimc */
    ar_tag_move_flag.data = false;
    artag_move_flag_pub.publish(ar_tag_move_flag);

    ar_tag_func_flag.data = 2; //Reset
    artag_func_flag_pub.publish(ar_tag_func_flag);
    
    ros::Duration(1.0).sleep();

    ar_tag_func_flag.data = 0; //Noraml
    artag_func_flag_pub.publish(ar_tag_func_flag);
    /* 210423jimc add end*/


    /* 210205jimc add end */
  }

  //got empty path as cancel type
  else if (goal_nums == 0)
  {
    LOG(INFO) << "get empty goal, as cancel cmd";
    robdir = -1;      /* 210504jimc */
     /* 210825jimc add */
    bay_request.data = 0;
    bay_alignment_pub.publish(bay_request);
    /* 210825jimc add end */
    AlarmCode = 0;
    emptycancel = true;
    ac.cancelAllGoals();
    ROS_INFO("[SLAM]cancel goal");
  }
  else
  {
    LOG(INFO) << "tracetype: " << (int)tracetype << ", goal_nums: " << goal_nums << ", gettag: " << gettag
                 << ", forwardstate: " << forwardstate << ", lastdynflag: " << lastdynflag;
    if (!gettag) //20210203wusl need to be modifie by ARTAG function
      AlarmCode = 7;  //MissingTagOnEndPoint
    else
      AlarmCode = 1;  //WrongReceivedMsg
  }

  //execute state end decision
  if (AlarmCode != 0 || emptycancel)
  {
    /* 210205jimc add start */
    if(tracetype == 1)
    {
      ar_tag_func_flag.data = 2; //Reset
      artag_func_flag_pub.publish(ar_tag_func_flag);
      /* 210427jimc add start */
      ros::Duration(1.0).sleep();

      ar_tag_func_flag.data = 0; //Noraml
      artag_func_flag_pub.publish(ar_tag_func_flag);
      /* 210427jimc add end */

      ar_tag_move_flag.data = false;
      artag_move_flag_pub.publish(ar_tag_move_flag);
      LOG(INFO) << "AR Tag Abnormal or emptycancel";
    }
    /* 210205jimc add end */

    robdir = -1;                      /* 210504jimc */
    misstion_exe_flg = 0;             /* 210408jimc */
    cmd_vel.linear.x = cmd_vel.linear.y = cmd_vel.angular.z = 0;
    current_cmd_vel.linear.x = current_cmd_vel.linear.y = current_cmd_vel.angular.z = 0;
    velocity_pub.publish(cmd_vel);
    //current task aborted
    dynflag = lastdynflag = 0;
    lastsegid = lastsubid = 0;
    if (emptycancel)
    {
      LOG(INFO) << "get empty cancel task ";
      as->setSucceeded();
    }
    else
    {
      LOG(INFO) << "moveserver tracking error";
      LOG(INFO) << "AlarmCode:" << AlarmCode;
      ROS_INFO("AlarmCode:%d",AlarmCode);
      /* 210408jimc as->setAborted(); */

      /* 210408jimc add */
      if( AlarmCode == 999 )
      {
        /* 210513jimc while( pausestop == 0 ) */
        while( (pausestop == 0) && (!as->isPreemptRequested()) )  /* 210513jimc */
        {
          if(forwardstop == 1)
          {
            as->setSucceeded();
            break;
          }
          LOG(INFO) << "Waiting replan..." ;
          ROS_INFO("Waiting replan...");
          ros::Duration(1.0).sleep();
        }
        /* 210513jimc add */
        if( (pausestop == 1) ||  (as->isPreemptRequested()) )
        {
          ros::Duration(1.0).sleep();
        }
        /* 210513jimc add end */
        as->setSucceeded();
      }
      else
      {
        as->setAborted();  
      }
      /* 210408jimc add end */
    }
  }
  else if (preemptreq == true)
  {
    if (forwardstop == 1)
    {
      LOG(INFO) << "get forward stop request set preempted";
      cmd_vel.linear.x = cmd_vel.linear.y = cmd_vel.angular.z = 0;
      current_cmd_vel.linear.x = current_cmd_vel.linear.y = current_cmd_vel.angular.z = 0;
      velocity_pub.publish(cmd_vel);
      //forwardstop = 0;  //0524 char mark test replan eror
      forwardstate = 1; //keep forwardstate for next time without check tag
      dynflag = lastdynflag = 0;
      lastsegid = lastsubid = 0;
      as->setSucceeded();
    }
    else
    {
      LOG(INFO) << "get new goal premmpted request";
      //current task aborted, and start with continue path request
      as->setSucceeded();
    }
    preemptreq = false;
  }
  else if (dynflag == 1 && tracetype == 0)
  {
    LOG(INFO) << "waitting cancel or continue goal";
    robdir = -1;      /* 210504jimc */
    as->setSucceeded();
  }
  else
  {
    LOG(INFO) << "moveserver get reached ";
    ROS_INFO("moveserver get reached");
    robdir = -1;      /* 210504jimc */
    dynflag = lastdynflag = 0;
    lastsegid = lastsubid = 0;
    current_cmd_vel.linear.x = current_cmd_vel.linear.y = current_cmd_vel.angular.z = 0;
    /* 210513jimc add */
    cmd_vel.linear.x = cmd_vel.linear.y = cmd_vel.angular.z = 0;
    velocity_pub.publish(cmd_vel);
    LOG(INFO) << "get reached distance:" << finalrodist_err ;
    ROS_INFO("get reached distance:%f",finalrodist_err);
    
    LOG(INFO) << "get reached angle:" << finalroangle_err*180/M_PI ;
    ROS_INFO("get reached angle:%f",finalroangle_err*180/M_PI);
    /* 210513jimc add end */
    as->setSucceeded();
  }
  allowpath = 1;
  //execute call back
}
/*0331 char check map score*/

void map_score_sub_CB(const std_msgs::Float32 &scoring)
{
  std_msgs::Bool mapupdate;
  map_score = scoring.data; /* 210513jimc */
  if(scoring.data <  map_score_limit  )
  {
    map_score_cunt++;
    
    if(map_score_cunt > 10)
    {
      remathstop =1;
      mapupdate.data =true;
      map_score_cunt =0;
    }

    /* 210913jimc add */
    map_score_low_count++;
    if(map_score_low_count > 50)
    {
      if( map_score_limit > map_score_low )
      {
        map_score_limit *= 0.9;
      }
      map_score_low_count = 0;
      ROS_INFO("Adjust map score to %f",map_score_limit);
    }
    /* 210913jimc add end */
  }
  else 
  {
    
    map_score_cunt= 0;
    remathstop =0;
    mapupdate.data =false;
  }
  if( (robdir == 1 )|| (robdir ==2))
  {
     mapupdate.data =false;
  }
  map_update_pub.publish(mapupdate);
}
/* 210205jimc add start */
void artag_stat_sub_CB(const artag_msgs::stat &msg)   //artag
{
  artagState = msg.stat;
  if( artagState == 1 )       // Moving
  {
    gettag = false;
    ROS_INFO("ARTag Moving");
    LOG(INFO) << "ARTag Moving";
  }
  else if( artagState == 2 )  // Finish
  {
    gettag = true;
    artagState = 0;
    ROS_INFO("ARTag Finish");
    LOG(INFO) << "ARTag Finish";
  }
  else if( artagState == 3 )  // Error
  {
    gettag = false;
    AlarmCode = 7;            // Missing tag on Endpoint"
    ROS_INFO("ARTag Error");
    LOG(INFO) << "ARTag Error";
  }
  /* 210427jimc add */
  else if( artagState == 4 )  // Timeout
  {
    gettag = false;
    AlarmCode = 7;            // Missing tag on Endpoint"
    ROS_INFO("ARTag Timeout");
    LOG(INFO) << "ARTag Timeout";
  }
  /* 210427jimc add end */
}
/* 210205jimc add end */


/* 210413jimc add start */
void obstacle_deceleration_status(const std_msgs::Int32 &stat)   //artag
{
  obsdecState = stat.data;
}
/* 210413jimc add end */

/* 210428jimc add */
void instrument_status(const std_msgs::Int32 &stat)   //Instrument Status
{
  instrumentState = stat.data;
}
/* 210428jimc add end */

/*
//barcode call back with debounce
void BarcodeReaderStateCB(const gpm_msgs::BarcodeReaderState::ConstPtr &msg)
{
  if (msg->state == 1) //tag
  {
    TagID = msg->tagID;
    gettag = true;
    gettap = false;
    tagcount = 0;

    tagxdis = (double)(msg->xValue) / 1000;
    tagydis = (double)(msg->yValue) / 1000;
    tagang = msg->theta * M_PI / 180;
    lastVisitedNode = msg->tagID;
  }
  else if( msg->state == 2) //tap
  {
    gettap = true;
    gettag = false;
    tapydis = (double)(msg->yValue) / 1000;
    tapang = msg->theta * M_PI / 180;
  }
  else
  {
    if (tagcount >= BcrRetryCount) // bcr_looprate=80,Setting to 40 means that reader haven't scanned any tag in 0.5s
    {
      TagID = msg->tagID;
      gettag = false;
      gettap = false;
    }
    else
      tagcount = tagcount + 1;
  }
}
*/
//service callback
bool ComplexCmdCB(gpm_msgs::ComplexRobotControlCmd::Request &req, gpm_msgs::ComplexRobotControlCmd::Response &res)
{
  LOG(INFO) << "Get ComplexRobotControlCmd : " << (int)req.reqsrv;
  ROS_INFO("Get ComplexRobotControlCmd:%d",(int)req.reqsrv);
  if (req.reqsrv == 0) // normal
  {
    pausestop = 0;
    isSlowDown = false;
    vel_max = speedmax;
    res.confirm = true;
    ROS_WARN("reset at 0");
  }
  else if (req.reqsrv == 1) //slow
  {
    isSlowDown = true;
    pausestop = 0;
    vel_max = vel_low;
    res.confirm = true;
    ROS_WARN("reset at 1");
  }
  else if (req.reqsrv == 2) //stop
  {
    isSlowDown = true;
    pausestop = 1;
    res.confirm = true;
    current_cmd_vel.linear.x =current_cmd_vel.linear.y =current_cmd_vel.angular.z =0.0;
    velocity_pub.publish(current_cmd_vel);
  }
  else if (req.reqsrv == 21) //secondary slow
  {
    vel_Secondary = vel_SecondarySlow;
    res.confirm = true;
  }
  else if (req.reqsrv == 100) //replan
  {
    forwardstop = 1;
    res.confirm = true;
  }
  else
    res.confirm = false;

  return true;
}
//
/* 210331jimc add */

bool Set_currentTagIDCB(gpm_msgs::SetcurrentTagID::Request &req, gpm_msgs::SetcurrentTagID::Response &res)
{
  int set_tagId = req.tagID;
  std::string set_map = req.map;
  double set_currentx = req.X;
  double set_currenty = req.Y;
  double set_currentangle = req.angle;

  LOG(INFO) << "Robot set_currentTagID : " << set_tagId;
  ROS_INFO("Robot set_currentTagID : %d",set_tagId);

  if (set_tagId != 0)
  {
    lastVisitedNode = set_tagId;
    res.confirm = true;
  }
  else
  {
    res.confirm = false;
    LOG(INFO) << "Robot set_currentTagID Error";
    ROS_INFO("Robot set_currentTagID Error");
  }

  return true;
}
/* 210331jimc add end */

/* 210401jimc add */
void obs_dec_mode_timer(const ros::TimerEvent&)
{
  //0: offline
	//1: corridor forward
	//2: corridor backward
	//3: bay forward
	//4: bay backward

  std_msgs::Int32 obs_dec_mode;
  double between_distance;

  /* 211008jimc if( ( misstion_exe_flg == 0 ) || ( AlarmCode != 0 ) || ( obsdec_avoid_flg == 1 ) ) */
  if( ( misstion_exe_flg == 0 ) || ( AlarmCode != 0 ) )/* 211008jimc */
  {
    //obstacle_deceleration_mode = 0;
    pausestop = false;
    //ROS_WARN("reset at other");
  }
  /*else
  {
    if( nav_param_select !=2 )
    {
      if(!back_flag)
      {
        obstacle_deceleration_mode = 1;
      }
      else
      {
        obstacle_deceleration_mode = 2;
      }
    }
    else
    {
      if(!back_flag)
      {
        obstacle_deceleration_mode = 3;
      }
      else
      {
        obstacle_deceleration_mode = 4;      
      }
    }
  }

  if( obstacle_deceleration_mode != pre_obstacle_deceleration_mode )
  {
    pre_obstacle_deceleration_mode = obstacle_deceleration_mode ;
    
    obs_dec_mode.data = obstacle_deceleration_mode;
    obstacle_deceleration_mode_pub.publish(obs_dec_mode);
    LOG(INFO) << "obstacle_deceleration_mode : " << obstacle_deceleration_mode;
    ROS_INFO("obstacle_deceleration_mode:%d",obstacle_deceleration_mode );
  }*/
  if( obsdec_avoid_flg == 1 )
  {
    planner_costmap_ros_->getRobotPose(obsdec_grobot_pose);
    between_distance = diserr(obsdec_lastgoal_pose, obsdec_grobot_pose);
    if(pre_obsdec_avoid_flg != obsdec_avoid_flg )
    {
      pre_obsdec_avoid_flg = obsdec_avoid_flg;
      LOG(INFO) << "obstacle deceleration between_distance : " << between_distance;
      ROS_INFO("obstacle deceleration between_distance : %.3f",between_distance );
    }
    if( between_distance > 2.5 )
    {
      LOG(INFO) << "obstacle deceleration between_distance : " << between_distance;
      ROS_INFO("obstacle deceleration between_distance : %.3f",between_distance );
      obsdec_avoid_flg = 0;
    }
  }
}
/* 210401jimc add end */

int main(int argc, char **argv)
{
  int delaypub = 0;
  google::SetLogDestination(google::GLOG_INFO, "/home/gpm/GPMLog/bcrmlog/info/barcodemove.INFO.");
  google::SetLogDestination(google::GLOG_WARNING, "/home/gpm/GPMLog/bcrmlog/warning/barcodemove.WARNING.");
  google::SetLogDestination(google::GLOG_ERROR, "/home/gpm/GPMLog/bcrmlog/error/barcodemove.ERROR.");
  google::InitGoogleLogging(argv[0]);
  fLI::FLAGS_max_log_size = 50;

  ros::init(argc, argv, "barcodemove_server");
  ros::NodeHandle n;
  ros::NodeHandle n_param("~");
  ros::Rate mloop(30);
  gpm_msgs::NavigationState navState;
  tf::Stamped<tf::Pose> tfgrobot_pose;
  geometry_msgs::PoseStamped robot_pose;

  /*-------------*/
  /*  Publisher  */
  /*-------------*/
  ros::Publisher state_pub = n.advertise<gpm_msgs::NavigationState>("navigation_state", 1);
  goal_pub = n.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 5);
  plan_pub = n.advertise<nav_msgs::Path>("global_plan", 5);                   //set path point to navigation
  velocity_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);            //send motor parameter to motor for move
  map_update_pub = n.advertise<std_msgs::Bool>("/map_update",1);                                    /* 210331char */
  map_switch_pub = n.advertise<std_msgs::Bool>("/map_switch",1);                                    /* 210421char */
  artag_tagID_pub = n.advertise<artag_msgs::artag_path>("/artag_path",1);                           /* 210205jimc */
  artag_move_flag_pub = n.advertise<std_msgs::Bool>("/lets_move_finished",1);                       /* 210205jimc */
  artag_func_flag_pub = n.advertise<std_msgs::UInt8>("/artag_func_flag",1);                         /* 210205jimc */
  obstacle_deceleration_mode_pub = n.advertise<std_msgs::Int32>("/obstacle_deceleration_mode",1);   /* 210331jimc */
  obst_mode_pub = n.advertise<std_msgs::Int32>("/obst_avoid_mode",1);   /* 2100927 */
  
  
  /*-------------*/
  /*  Subscribe  */
  /*-------------*/
  //ros::Subscriber bcr_sub = n.subscribe("barcodereader_state", 1, BarcodeReaderStateCB);
  map_score_sub = n.subscribe("map_score",1,map_score_sub_CB);
  slam_cmd_vel_sub = n.subscribe("/slam_cmd_vel", 10, &slam_cmd_vel_callBack);    //navigation reply motor parameter
  goal_result_sub = n.subscribe("/move_base/result", 1, &result_callBack);
  robot_pose_now_sub =  n.subscribe("/robot_pose_now", 1, &robot_pose_now_callBack);  //0427 char 
  bay_alignment_pub = n.advertise<std_msgs::Int16>("/bay_request", 1);          /* 210511GU */
  artag_stat_sub = n.subscribe("stat",10,artag_stat_sub_CB);                                         /* 210205jimc */
  vel_sub = n.subscribe("/cmd_vel", 1000, &velCallback);                                            /* 210324jimc */
  //obstacle_deceleration_status_sub = n.subscribe("/obstacle_deceleration_status",1,obstacle_deceleration_status);   /* 210413jimc */
  instrument_status_sub = n.subscribe("/instrument_status",1,instrument_status);                    /* 210428jimc */
  laser_sub = n.subscribe("/scan", 1, &laser_callback);                                             //20210330GU
  bay_response_sub = n.subscribe("/bay_response", 5, &bay_response_callback);                       /* 210511GU */
  velctrl_sub = n.subscribe("/obst_velctrl", 1, &velctrl_callback);     //0927 char                                        //20210330GU
  TEB_path_sub =  n.subscribe("/TEB_pose_tf", 1, &TEB_path_tf_callBack);  //1104 char  /* teb_path */

  /*------------------*/
  /*  Service server  */
  /*------------------*/
  ros::ServiceServer comlex_cmd_req = n.advertiseService("complex_robot_control_cmd", ComplexCmdCB);
  ros::ServiceServer set_currentTagID_req = n.advertiseService("set_currentTagID", Set_currentTagIDCB);  /* 210331jimc */
  
  /*------------------*/
  /*  Service client  */
  /*------------------*/
  switch_map_client = n.serviceClient<gpm_msgs::PoseStamped>("map_switch_info");  /* 210409jimc */
  ros::ServiceClient clear_client = n.serviceClient<std_srvs::Empty>("/move_base/clear_costmaps");
  std_srvs::Empty srv;


  /*-----------*/
  /*   Timer   */
  /*-----------*/
  ros::Timer obsdec_mode_timer = n.createTimer(ros::Duration(0.1), &obs_dec_mode_timer, false);  /* 210401jimc */

  /*------------*/
  /*   Action   */
  /*------------*/
  Server server(n, "barcodemovebase", boost::bind(&execute, _1, &server), false);

  tf::TransformListener tflistener(ros::Duration(5));
  tf::TransformListener &tf_ = tflistener;
  
  planner_costmap_ros_ = new costmap_2d::Costmap2DROS("global_costmap", tf_);
  planner_costmap_ros_->pause();
  controller_costmap_ros_ = new costmap_2d::Costmap2DROS("local_costmap", tf_);
  controller_costmap_ros_->pause();
  planner_costmap_ros_->start();
  controller_costmap_ros_->start();
  
  server.start();

  ReadParam();  
  n_param.param<int>("lastVisitedNode", lastVisitedNode, 100);
  //n_param.param<float>("map_score_limit", map_score_limit, 0.5);
  n_param.param<float>("map_score_high", map_score_high, 0.75);
  n_param.param<float>("map_score_low", map_score_low, 0.5);
  map_score_limit=map_score_high;
  int time_count =0;
  system("rosrun dynamic_reconfigure dynparam load /barcodemove/global_costmap /home/gpm/catkin_ws/src/gpm_project/barcodemove/param/barcode_costmap.yaml &");  
  system("rosrun dynamic_reconfigure dynparam load /barcodemove/local_costmap /home/gpm/catkin_ws/src/gpm_project/barcodemove/param/barcode_costmap.yaml &");  

  while (ros::ok())
  {

    robot_pose.header.stamp = ros::Time::now();
    robot_pose.header.frame_id = "map";
    
    planner_costmap_ros_->getRobotPose(tfgrobot_pose);
    
    poseStampedTFToMsg(tfgrobot_pose, robot_pose);
    
    navState.lastVisitedNode.data = lastVisitedNode; //20210203wusl
    navState.robotPose = robot_pose;
    navState.robotDirect = robdir;
    navState.newPathAllowed = allowpath;
    navState.errorCode = AlarmCode;
    navState.comparisonRate = map_score*100;

    state_pub.publish(navState);
    //0504 char add clear costmap at not avoidance
    
    //if(nav_param_select != 1)
    {
      if(time_count > 10 *30)
      {
        clear_client.call(srv);
        //ROS_INFO("clear test");
        time_count =0;
      }
      else
      {
        time_count ++;
      }
    }
    
    //0504 char add end
    
    ros::spinOnce();
    mloop.sleep();
  }
  google::ShutdownGoogleLogging();
  return 0;
}
