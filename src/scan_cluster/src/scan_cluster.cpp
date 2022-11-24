#include <ros/ros.h>

#include "points_cutter.h"
#include "dbscan.h"

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <string>
#include <ctime>

#define _NODE_NAME_ "scan_cluster"

using namespace std;

class Scan_cluster{
private:
	
	double left_edge_, right_edge_;
	double front_edge_, behind_edge_;
	double max_distance_, left_angle_, right_angle_;
	double Eps_;
	int Minpts_, MaxPts_;
	bool show_points_;
	
	string cluster_pub_topic_;
	ros::Publisher cluster_pub_;
	
	ros::Publisher points_pub_;
	
	string scan_sub_topic_;
	ros::Subscriber scan_sub_;
	
	void scancallback(const sensor_msgs::LaserScan msg);
	vector<point> points_;
	vector<point> clusters_;
	
public:
	Scan_cluster();
	void showPoints();
	void showClusters();
};

Scan_cluster::Scan_cluster()
{
	ros::NodeHandle nh, nh_private("~");
	nh_private.param<std::string>("scan_sub_topic", scan_sub_topic_, "scan");
	nh_private.param<std::string>("cluster_pub_topic", cluster_pub_topic_, "clusters");
	nh_private.param<double>("left_edge", left_edge_, 100);
	nh_private.param<double>("right_edge", right_edge_, 100);
	nh_private.param<double>("front_edge", front_edge_, 100);
	nh_private.param<double>("behind_edge", behind_edge_, 100);
	nh_private.param<double>("max_distance", max_distance_, 100);
	nh_private.param<double>("left_angle", left_angle_, 0);
	nh_private.param<double>("right_angle", right_angle_, 0);
	nh_private.param<double>("Eps", Eps_, 0.5);
	nh_private.param<int>("Minpts", Minpts_, 20);
	nh_private.param<int>("MaxPts", MaxPts_, 20);
	nh_private.param<bool>("show_points", show_points_, false);
	
	cout << "sub topic: " << scan_sub_topic_ << '\n'
		 << "pub_topic: " << cluster_pub_topic_ << '\n'
		 << "left edge: " << left_edge_ << '\n'
		 << "right edge: " << right_edge_ << '\n'
		 << "front edge: " << front_edge_ << '\n'
		 << "behind edge: " << behind_edge_ << '\n'
		 << "Eps: " << Eps_ << '\n'
		 << "Minpts: " << Minpts_ << '\n'
		 << "Maxpts: " << Minpts_ << '\n'
		 << "show points: " << show_points_ << endl;
	
	scan_sub_ = nh_private.subscribe(scan_sub_topic_ ,2,&Scan_cluster::scancallback, this);
	if (show_points_)
		points_pub_ = nh_private.advertise<visualization_msgs::MarkerArray>("/points", 1);
	cluster_pub_ =  nh_private.advertise<visualization_msgs::MarkerArray>(cluster_pub_topic_, 1);
}

void Scan_cluster::scancallback(const sensor_msgs::LaserScan msg)
{
	clock_t st = clock();
	Scan_cutter scan_cutter(msg);
	scan_cutter.setEdge(left_edge_, right_edge_, front_edge_, behind_edge_);
	//scan_cutter.setEdge(max_distance_, left_angle_, right_angle_);
	
	if (!scan_cutter.points_cut())
	{
		cout << "error when cutting!" << endl;
		return;
	}
	points_ = scan_cutter.getPoints();
	
	if (show_points_)
		showPoints();
	
	clusters_ = DBSCAN(points_, Eps_, Minpts_, MaxPts_);
	//cout << "clusters: " << clusters_.size() << endl;
	showClusters();
	
	clock_t ed = clock();
	double t = (double)(ed-st)/CLOCKS_PER_SEC;
	cout << "cluster done!    time cost: " << t << endl;
}

void Scan_cluster::showPoints()
{
	cout << "publishing points : " << points_.size() << endl;
	visualization_msgs::MarkerArray pts;
	for (int i=0; i<points_.size(); ++i)
	{
		visualization_msgs::Marker pt;
		
		pt.header.frame_id = "laser_link";
		pt.header.stamp = ros::Time::now();
		
		pt.id = i;
		pt.ns = "point";
		pt.type = visualization_msgs::Marker::CYLINDER;
		pt.action = visualization_msgs::Marker::ADD;
		
		pt.pose.position.x = points_[i].x;
		pt.pose.position.y = points_[i].y;
		pt.pose.position.z = 0;
		
		pt.scale.x = 0.03;
		pt.scale.y = 0.03;
		pt.scale.z = 0.05;
		
		pt.color.r = 0.0f;
		pt.color.g = 1.0f;
		pt.color.b = 0.0f;
		pt.color.a = 1.0f;
		
		pt.lifetime = ros::Duration(0.1);
		pts.markers.push_back(pt);
	}
	
	points_pub_.publish(pts);
}

void Scan_cluster::showClusters()
{
	visualization_msgs::MarkerArray clusters;
	for (int i=0; i<clusters_.size(); ++i)
	{
		visualization_msgs::Marker clu;
		
		clu.header.frame_id = "laser_link";
		clu.header.stamp = ros::Time::now();
		
		clu.id = i;
		clu.ns = "cluster";
		clu.type = visualization_msgs::Marker::CYLINDER;
		clu.action = visualization_msgs::Marker::ADD;
		
		clu.pose.position.x = clusters_[i].x;
		clu.pose.position.y = clusters_[i].y;
		clu.pose.position.z = 0;
		
		clu.scale.x = clusters_[i].r;
		clu.scale.y = clusters_[i].r;
		clu.scale.z = 0.05;
		
		clu.color.r = 0.0f;
		clu.color.g = 1.0f;
		clu.color.b = 0.0f;
		clu.color.a = 1.0f;
		
		clu.lifetime = ros::Duration(0.1);
		clusters.markers.push_back(clu);
	}
	cluster_pub_.publish(clusters);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, _NODE_NAME_);
	Scan_cluster scan_cluster;
	ros::spin();
	return 0;
}
