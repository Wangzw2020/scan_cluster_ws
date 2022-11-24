#ifndef POINTS_CUTTER_H
#define POINTS_CUTTER_H

#include <ros/ros.h>
#include <iostream>
#include <sensor_msgs/LaserScan.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <vector>

using namespace std;

struct point{
	double x;
	double y;
	double r = 0;
};

class Scan_cutter{
private:
	sensor_msgs::LaserScan scan_points_;
	vector<point> points_;
	double left_edge_, right_edge_;
	double front_edge_, behind_edge_;
	double max_distance_, left_angle_, right_angle_;
	
public:
	Scan_cutter(){};
	~Scan_cutter(){};
	Scan_cutter(const sensor_msgs::LaserScan msg);
	void setEdge(double l, double r, double f, double b);
	void setEdge(double max, double l, double r);
	bool points_cut();
	vector<point> getPoints() { return points_; }
	int getNumPoints() { return points_.size(); }

};

Scan_cutter::Scan_cutter(const sensor_msgs::LaserScan msg)
{
	scan_points_ = msg;
}

void Scan_cutter::setEdge(double l, double r, double f, double b)
{
	left_edge_ = l; right_edge_ = r;
	front_edge_ = f; behind_edge_ = b;
}

void Scan_cutter::setEdge(double max, double l, double r)
{
	max_distance_ = max;
	left_angle_ = l;
	right_angle_ = r;
}

bool Scan_cutter::points_cut()
{
	vector<float> ranges = scan_points_.ranges;
	
	for (int i=0; i<ranges.size(); ++i)
	{
		double angle = scan_points_.angle_min + i * scan_points_.angle_increment;
		double X, Y;
		X = ranges[i] * cos(angle);
		Y = ranges[i] * sin(angle);
		
		if (X < behind_edge_ || X > front_edge_)
			continue;
		if (Y < left_edge_ || Y > right_edge_)
			continue;
		if (X == 0 && Y == 0)
			continue;
		point a;
		a.x = X;
		a.y = Y;
		points_.push_back(a);
	}

	return true;
}



#endif
