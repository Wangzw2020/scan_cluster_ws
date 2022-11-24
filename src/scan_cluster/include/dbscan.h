#ifndef DBSCAN_H
#define DBSCAN_H

#include <ros/ros.h>
#include "points_cutter.h"
#include <stack>
#include <vector>
#include <cmath>

class DBpoint{
public:
	double x_, y_;
	int cluster_;
	int pointType_;		//1 noise 2 border 3 core
	int pts_;			//points in MinPts
	vector<int> corepts_;
	bool visited_;

public:
	DBpoint(){}
	DBpoint(double x, double y, int i);
};

double squareDistance(DBpoint a,DBpoint b)
{
	return sqrt((a.x_ - b.x_) * (a.x_ - b.x_) + (a.y_ - b.y_) * (a.y_ - b.y_));
}

vector<point> DBSCAN(vector<point> pts, double Eps, int MinPts, int MaxPts)			//!
{
	//cout << "DBSCAN!" << endl;
	
	vector<DBpoint> dataset;
	for(int i=0; i<pts.size(); ++i)
	{
		DBpoint a(pts[i].x, pts[i].y, i);
		dataset.push_back(a);
	}
	
	//cout << "size of dataset: " << dataset.size() << endl;
	
	for(int i=0; i<dataset.size(); ++i)
	{
		for(int j=i+1; j<dataset.size(); ++j)
		{
			if(squareDistance(dataset[i],dataset[j]) < Eps)
			{
				dataset[i].pts_++;
				dataset[j].pts_++;
			}
		}
	}
	
	vector<DBpoint> corePoints;
	for(int i=0; i<dataset.size(); ++i)
	{
		if(dataset[i].pts_ >= MinPts)
		 {
			dataset[i].pointType_ = 3;
			corePoints.push_back(dataset[i]);
		}
	}
	
	//cout << "size of corePoints: " << corePoints.size() << endl;
	
	for(int i=0; i<corePoints.size(); ++i)
	{
		for(int j=i+1; j<corePoints.size(); ++j)
		{
			if(squareDistance(corePoints[i],corePoints[j]) < Eps)
			{
				corePoints[i].corepts_.push_back(j);
				corePoints[j].corepts_.push_back(i);
			}
		}
	}
	
	for(int i=0; i<corePoints.size(); ++i)
	{
		stack<DBpoint*> ps;
		
		if(corePoints[i].visited_ == true)
			continue;
			
		ps.push(&corePoints[i]);
		DBpoint *v;
		while(!ps.empty())
		{
			v = ps.top();
			v->visited_ = true;
			ps.pop();
			for(int j=0; j<v->corepts_.size(); ++j)
			{
				if(corePoints[v->corepts_[j]].visited_ == true)
					continue;
				corePoints[v->corepts_[j]].cluster_ = corePoints[i].cluster_;
				corePoints[v->corepts_[j]].visited_ = true;
				ps.push(&corePoints[v->corepts_[j]]);
			}
		}
	}
	
	for(int i=0; i < dataset.size(); ++i)
	{
		if(dataset[i].pointType_ == 3)
			continue;
		for(int j=0; j<corePoints.size(); ++j)
		{
			if(squareDistance(dataset[i], corePoints[j]) < Eps) 
			{
				dataset[i].pointType_ = 2;
				dataset[i].cluster_ = corePoints[j].cluster_;
				break;
			}
		}
	}
	
	vector<vector<DBpoint>> cluster_res;
	
	for (int i=0; i<corePoints.size(); ++i)
	{
		bool have=false;
		for (int j=0; j<cluster_res.size(); ++j)
		{
			if (corePoints[i].cluster_ == cluster_res[j][0].cluster_)
			{
				have = true;
				cluster_res[j].push_back(corePoints[i]);
				break;
			}
		}

		if (have == true)
			continue;
		vector<DBpoint> newcluster;
		newcluster.push_back(corePoints[i]);
		cluster_res.push_back(newcluster);
	}
	
	vector<point> clusters;
	for (int i=0; i<cluster_res.size(); ++i)
	{

		if (cluster_res[i].size() > MaxPts)
			continue;
		
		double totalx = 0, totaly = 0;
		point pt;
		pt.r = 0;
		for (int k=0; k<cluster_res[i].size(); ++k)
		{
			totalx += cluster_res[i][k].x_;
			totaly += cluster_res[i][k].y_;
		}
		pt.x = totalx / cluster_res[i].size();
		pt.y = totaly / cluster_res[i].size();
		
		for (int k=0; k<cluster_res[i].size(); ++k)
		{
			double tmp_r = sqrt((cluster_res[i][k].x_ - pt.x) * (cluster_res[i][k].x_ - pt.x) + (cluster_res[i][k].y_ - pt.y) * (cluster_res[i][k].y_ - pt.y));
			if(tmp_r > pt.r)
				pt.r = tmp_r;
		}
		clusters.push_back(pt);
	}
	
	return clusters;
}

DBpoint::DBpoint(double x, double y, int i)
{
	cluster_ = 0;
	pointType_ = 1;
	pts_ = 0;
	visited_ = false;
	x_ = x;
	y_ = y;
	cluster_ = i;
}

#endif
