#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Int8.h>
#include <tf/transform_datatypes.h>
#include <signal.h>
#include <math.h>
#include <vector>
#include <algorithm>
#include <typeinfo>
using namespace std;
const int maxnum = 20;
const int maxint = 999999;
int start_node,end_node;
bool start_flag,end_flag;
void turn(float goal_angle);
void searchPath(int *prev,int v, int u,ros::Publisher path_pub);
void Dijkstra(int n, int v, int *dist, int *prev, int c[maxnum][maxnum]);
void int_Callback(const std_msgs::Int8::ConstPtr& msg){
	start_flag=true;
	start_node=msg->data;
  std::cout<<"I receive start_node "<<start_node<<", please set next node "<<std::endl;
}

class Param
{
	public:	
	std::vector<float> waypoint;
	void end_node_Callback(const geometry_msgs::PoseStamped& end);
};

void Param::end_node_Callback(const geometry_msgs::PoseStamped& end){
	
	std::vector<float> dist;
	end_flag=true;
	std::cout<<"end x 座標 "<< end.pose.position.x << "end y 座標 " << end.pose.position.y << std::endl;
	for(int i=1;i<waypoint.size()/4;i++){
		dist.push_back(sqrt(pow(end.pose.position.x-waypoint[i*4+0],2)+
							pow(end.pose.position.y-waypoint[i*4+1],2)+
							pow(end.pose.orientation.z-waypoint[i*4+2],2)+
							pow(end.pose.orientation.w-waypoint[i*4+3],2)));
		std::cout<<"dist "<<i-1<<" "<<dist[i-1]<<"  "<<std::endl;
	}
	auto smallest=std::min_element(std::begin(dist), std::end(dist));
	//auto a=std::distance(std::begin(dist), smallest);
	end_node=static_cast<int>(distance(begin(dist), smallest))+1;
	std::cout<<"min element is "<<*smallest<<"at position "<<end_node<<std::endl;
	dist.clear();
}


int main(int argc, char** argv)
{ 
  ros::init(argc, argv, "auto_path");
  ros::NodeHandle nh;
  ros::NodeHandle nh_priv("~");
  ros::Rate loop_rate(10);
  ros::Duration d(0.1);
Param param;
// from launch get parameters
  int n,line; 
  nh_priv.param("nodenum", n,int());
  nh_priv.param("linenum", line,int());
  std::vector<int> p;
  nh_priv.param("p", p, std::vector<int>());
  std::vector<int> q;
  nh_priv.param("q", q, std::vector<int>());
  std::vector<int> len;
  nh_priv.param("len", len, std::vector<int>());
  //std::vector<float> waypoint;
  nh_priv.param("waypoint", param.waypoint, std::vector<float>());
	line=p.size()-1;

  ros::Publisher path_pub=nh.advertise<std_msgs::Int16MultiArray>("path",10);
  ros::Subscriber int_sub = nh.subscribe("start_node", 2, int_Callback); 
  ros::Subscriber end_node_sub = nh.subscribe("move_base_simple/temp_goal",2,&Param::end_node_Callback,&param);  


start_flag=1;
	int dist[maxnum];     // 表示當前點到源點的最短路徑長度
	int prev[maxnum]={maxint};     // 記錄當前點的前一個結點
	int c[maxnum][maxnum]={0};   // 記錄圖的兩點間路徑長度
	std::cout<<"set start node \n";
	cin>>start_node;
while(ros::ok()){	
	ros::Duration(0.1).sleep();
	ros::spinOnce();

	if(!start_flag || !end_flag)continue;	
	c[maxnum][maxnum]={0};
	dist[maxnum]={0};
	int prev[maxnum]={maxint};
	for(int i=1; i<=n; ++i)
		for(int j=1; j<=n; ++j)
			c[i][j] = maxint;
 
	for(int i=1; i<=line; ++i)  
	{
		int a=p[i],b=q[i];
		if(len[i] < c[a][b])       // 有重邊
		{
			c[a][b] = len[i];      // p指向q
			//c[b][a] = len[i];      // q指向p，這樣表示無向圖
		}
	}
 
	for(int i=1; i<=n; ++i)
		dist[i] = maxint;
	for(int i=1; i<=n; ++i)
	{
		for(int j=1; j<=n; ++j)
			printf("%8d", c[i][j]);
		printf("\n");
	}

  Dijkstra(n, start_node, dist, prev, c);


	// 路徑
	cout <<"from "<<start_node<<" to "<<end_node<< " 路徑為: ";
	searchPath(prev, start_node, end_node,path_pub);

	start_flag=false;
	end_flag=false;
	}

  return 0;
}

void Dijkstra(int n, int v, int *dist, int *prev, int c[maxnum][maxnum])
{
	bool s[maxnum];    // 判斷是否已存入該點到S集合中
	for(int i=1; i<=n; ++i)
	{
		dist[i] = c[v][i];
		s[i] = 0;     // 初始都未用過該點
		if(dist[i] == maxint)
			prev[i] = 0;
		else
			prev[i] = v;
	}
	dist[v] = 0;
	s[v] = 1;
 
	// 依次將未放入S集合的結點中，取dist[]最小值的結點，放入結合S中
	// 一旦S包含了所有V中頂點，dist就記錄了從源點到所有其他頂點之間的最短路徑長度
         // 註意是從第二個節點開始，第一個為源點
	for(int i=2; i<=n; ++i)
	{
		int tmp = maxint;
		int u = v;
		// 找出當前未使用的點j的dist[j]最小值
		for(int j=1; j<=n; ++j)
			if((!s[j]) && dist[j]<tmp)
			{
				u = j;              // u保存當前鄰接點中距離最小的點的號碼
				tmp = dist[j];
			}
		s[u] = 1;    // 表示u點已存入S集合中
 
		// 更新dist
		for(int j=1; j<=n; ++j)
			if((!s[j]) && c[u][j]<maxint)
			{
				int newdist = dist[u] + c[u][j];
				if(newdist < dist[j])
				{
					dist[j] = newdist;
					prev[j] = u;
				}
			}
	}
}
 
// 查找從源點v到終點u的路徑，並輸出
void searchPath(int *prev,int v, int u,ros::Publisher path_pub)
{
	std_msgs::Int16MultiArray msg;
	ros::Duration(0.1).sleep();

	int que[maxnum];
	int tot = 1;
	que[tot] = u;
	tot++;
		
	int tmp = prev[u];
	while(tmp != v)
	{
		que[tot] = tmp;
		tot++;
		tmp = prev[tmp];
	}
	que[tot] = v;
	for(int i=tot; i>=1; --i){
		msg.data.push_back(que[i]);
		if(i != 1)
			cout << que[i] << " -> ";
						
		else
			cout << que[i] << endl;
	}
	for(int i=0;i<msg.data.size();i++)
		std::cout<<"pub_msg is "<<msg.data[i]<<std::endl;
	
	path_pub.publish(msg);
	//ros::Duration(0.1).sleep();
}