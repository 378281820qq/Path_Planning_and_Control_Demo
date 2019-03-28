#ifndef  RRT_H_HH_H
#define  RRT_H_HH_H
#include <vector>
#include <iostream>
#include <opencv2/opencv.hpp>
#include "map_pgm.h"

struct point {
	int x;
	int y;
//	std::vector<point > next;
	point(int x=0,int y=0){this->x=x;this->y=y;}
	point(const point &p){this->x=p.x;this->y=p.y;}
};

struct tree_node{
	tree_node * father;
	point pos;
	tree_node(point pos,tree_node * parent=NULL){this->pos=pos;father=parent;}
	tree_node& operator=(const tree_node &node){this->pos=node.pos;this->father=node.father;}
};

class RRT{
public:
	
	RRT(const char * filename,point start,point goal);
	void createTree();
	
	bool getPath();
	void showResult(const char * filename);
	//void repair();
	~RRT();

private:
	int getDistance(const point p1,const point p2);
	bool if_line(const point a,const point b);
	const int K; //K 撒点数目
	const int minstep; // minstep树节点之间的最短距离
	std::vector<point > line;
	std::vector<point> allnode;
	//	graph G;
	Pgm_map basemap;
	std::vector<tree_node *> graph;  //RRT顶点集合
	std::vector<point> waypoints;

	point start;
	point goal;
	int ** map;
	cv::Mat img;
	int sizeX,sizeY;	
};

#endif