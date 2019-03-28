#include "RRT.h"
#include <opencv2/opencv.hpp>
#include <cstdlib>
#include <ctime>
#include <cmath>
#include <fstream>


RRT::RRT(const char * filename,point start,point goal):K(10000),basemap(filename),minstep(20){		//K 撒点数目， minstep树节点之间的最短距离
	using namespace cv;
	allnode.push_back(start);
	tree_node * t=new tree_node(start);
	graph.push_back(t);
	img= imread(filename);
	this->start=start;
	this->goal=goal;
	if (basemap.transformMapToGrid()){
		map=basemap.getGridMap2D();
		sizeX=basemap.getSizeX();
		sizeY=basemap.getSizeY();
		std::cout << "sizeX    " << sizeX << "   sizeY   " << sizeY <<std::endl;
	}	
}


int RRT::getDistance(const point p1,const point p2){
	return (int)sqrt(double((p1.x-p2.x)*(p1.x-p2.x)+(p1.y-p2.y)*(p1.y-p2.y)));
}


//点a和点b是否可以连接成一条直线，中间没有障碍物
bool RRT::if_line(const point a,const point b){
	int ** walkability = map;
	line.clear();
	int dx=abs(a.x-b.x);
	int dy=abs(a.y-b.y);
	double k;
	if (dx != 0){
		k=double(b.y-a.y)/double(b.x-a.x);
	}
	else if (dy ==0){
		k=1;
	}
	else{
		k=99999;
	}
	
	point temp;

	if (dx>=dy)	{
		if (a.x<b.x){
			temp =a;
		}
		else {
			temp = b;
		}

		for (int i=1;i<dx+1;i++){
			int x=temp.x+i;
			int y=temp.y+k*i;
			point pt(x,y);
			line.push_back(pt);
		}
	}
	else
	{
		if (a.y<b.y)
		{
			temp =a;
		}
		else 
			temp=b;
		for (int i=1;i<dy+1;i++)
		{
			int x=temp.x+i/k;
			int y=temp.y+i;

			point pt(x,y);
			line.push_back(pt);
		}
	}
	int len=line.size();
	for (int i=0;i<len;i++)	{
		if (1 == walkability[line[i].x][line[i].y])//是否占据
		{
			return false;
		}
	}
	return true;
}




void RRT::createTree(){
	using namespace std;
	using namespace cv;
	int i=0;
	srand(time(0)); //使用当前时间作为随机种子数
	bool findwaypoints=true;
	while (i++ < K)	{   //K 撒点数目		
		int x=rand()%sizeX;
		int y=rand()%sizeY;
		//cout <<x << endl;
		
		while(map[x][y] != 0){  //available
			x=rand()%sizeX;
			y=rand()%sizeY;
			//cout <<x << endl;
		}

		point temp(x,y);  //随机的点
		int flag=0;
		int dist=999999;
		point p;

		if (findwaypoints){  //未达到目标
			for (int i=0;i<graph.size();i++){ 
				int d=getDistance(temp,graph[i]->pos);
				if (d< dist){
					dist=d;
					flag=i;
				}
			}
			p.x=graph[flag]->pos.x;
			p.y=graph[flag]->pos.y;  //距离temp最近的点
		}

		else{				//达到目标
			for (int i=0;i<allnode.size();i++){
				int d = getDistance(temp,allnode[i]);
				if (d< dist){
					dist=d;
					flag=i;
				}
			}
			p.x=allnode[flag].x;
			p.y=allnode[flag].y;
		}
				
		int num=9999;
		int pos=0;
		while ( ! if_line(p,temp) && num >minstep){   //minstep树节点之间的最短距离
				num=line.size();
				pos = rand()%num;
				num=pos;
				temp=line[pos];
		}
		num=line.size();

		if (num < minstep || ! if_line(p,temp))	{
			continue;
		}

		for (int i=0;i<num;i++)	{
			circle( img,Point(line[i].x,line[i].y), 0.1,  Scalar(0,10,255), 0.1, 1,0 );
		}

		if (num != 0){
			if (findwaypoints){
				tree_node * t=new tree_node(temp,graph[flag]);  //graph[flag]为parent
				graph.push_back(t);    //加入RRT的顶点集合中
			}
			
			
			if (findwaypoints && (getDistance(temp,goal) < minstep+10 ) && if_line(temp,goal)){
				findwaypoints=false; //达到目标
				waypoints.push_back(goal);
				tree_node * temp_ = graph[graph.size()-1];
				circle( img,Point(start.x,start.y), 3,  Scalar(0,0,0), 3, 1, 0 );
				circle(img, Point(goal.x, goal.y), 3, Scalar(0, 0, 0), 3, 1, 0);
				string str = "start";
				putText(img, str, Point(start.x, start.y), cv::FONT_HERSHEY_DUPLEX, 2, cv::Scalar(0, 255, 0), 2);				
				string str_1 = "goal";
				putText(img, str_1, Point(goal.x, goal.y), cv::FONT_HERSHEY_DUPLEX, 2, cv::Scalar(0, 255, 0), 2);


				for (;temp_!=NULL;temp_=temp_->father){
					waypoints.push_back(temp_->pos);
				}
			}
			allnode.push_back(temp);
		}
	}	
}


// plot waypoints with lines in image
bool RRT::getPath(){
	using namespace cv;
	std::cout << "waypoints.size()    " << waypoints.size() <<std::endl; //30
	if (waypoints.size()== 0)	{
		return false;
	}
	else{
			for (int i=0;i<waypoints.size()-1;i++){
				cv::line( img, Point(waypoints[i].x,waypoints[i].y), Point(waypoints[i+1].x,waypoints[i+1].y), Scalar(255,0,0),2 );
		}
		return true;
	}
}

// image show
void RRT::showResult(const char * filename) {
	using namespace std;
	using namespace cv;
	namedWindow("map", CV_WINDOW_NORMAL);//鼠标调整窗口的大小
	//resizeWindow("map", 640, 480);
	imshow("map", img);
	imwrite(filename, img);
	waitKey();
}

RRT::~RRT(){
	for (int i=1;i<graph.size()-1;i++){
		delete graph[i];
	}
}

/*
void RRT::repair()
{
	

}

*/