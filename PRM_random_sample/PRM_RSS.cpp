#include "PRM_RSS.h"
#include <cstdlib>
#include <ctime>
#include <cmath>
#include <opencv2/opencv.hpp>
#include "Astar.h"
#include <fstream>


//����ײ����һ��ֱ����
bool PRM_RSS_map::if_line(const point a,const point b){
	int ** walkability = buff;
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
		else 
			temp=b;
		for (int i=1;i<dx+1;i++){
			int x=temp.x+i;
			int y=temp.y+k*i;
			point pt(x,y);
			line.push_back(pt);
		}
	}
	else{
		if (a.y<b.y){
			temp =a;
		}
		else 
			temp=b;
		for (int i=1;i<dy+1;i++){
			int x=temp.x+i/k;
			int y=temp.y+i;
			point pt(x,y);
			line.push_back(pt);
		}
	}
	int len=line.size();
	for (int i=0;i<len;i++)	{
		if (1 == walkability[line[i].x][line[i].y])	{
			return false;
		}
	}
	return true;
}






int PRM_RSS_map::getDistance(const point p1,const point p2){
	return (int)sqrt(double((p1.x-p2.x)*(p1.x-p2.x)+(p1.y-p2.y)*(p1.y-p2.y)));
}


//���캯��
PRM_RSS_map::PRM_RSS_map(const char * filename,point start,point goal):map(filename),Rmax(50),K(1200),Rmin(10){	// K ���������� Rmin�������������룬 Rmax���������Զ���� 
	using namespace cv;
	using namespace std;
	img=imread(filename);
	srand(time(0));
	this->start=start;
	this->goal=goal;
	if (map.transformMapToGrid()){
		sizeX=map.getSizeX();
		sizeY=map.getSizeY();
		buff=map.getGridMap2D();
		bool findpath=true;
		int k=0;
		
		p_node* t=new p_node(start);
		allnode.push_back(t);
		while (k++ < K){
			int x,y;
			x=rand()%sizeX;
			y=rand()%sizeY;

			while(buff[x][y] != 0){
				x=rand()%sizeX;
				y=rand()%sizeY;
			}

			point temp(x,y);  //�����
			p_node* te= new p_node(temp);			
			circle( img,Point(temp.x,temp.y), 1 ,  Scalar(0,0,0), 3, 1, 0 );  //ͼ�ϻ��������
			
			int dmax=999999;
			int flag=-1;

			for (int i=0;i<allnode.size();i++){ //�����ٽ���
				int d=getDistance(temp,allnode[i]->pose);
				if (d<Rmax && d>Rmin){
					if (if_line(temp,allnode[i]->pose)){
						te->neighbors.push_back(allnode[i]);
						allnode[i]->neighbors.push_back(te);
						for (int i=0;i<line.size();i++){
							circle( img,Point(line[i].x,line[i].y), 0.1,  Scalar(0,10,255), 0.1, 1, 0 );
						}
					}
				}
			}//�����ٽ��㣬����
			allnode.push_back(te);	
		} //�������
	}
}


bool PRM_RSS_map::getPath(){
	using namespace cv;
	Astar pathfinder;

	p_node * g=NULL;
	int dis=99999;
	for (int i=0;i<allnode.size();i++){   //ȡ��goal�����һ���ڵ�
		int d = getDistance(allnode[i]->pose,goal);
		if (dis > d	){
			dis = d;
			g=allnode[i];
		}
	}

	if (  (g==NULL) || !(if_line(g->pose,goal))  ) { //û����goal�����һ���ڵ������·����ͨ
		std::cout<<"path find failed!"<<std::endl;
		return false;
	}

	pathfinder.getPath(allnode[0],g);   //bool getPath(p_node* start,p_node* goal);

	if (pathfinder.path.size() == 0){
		std::cout<<"path find failed!"<<std::endl;
		return false;
	}else{
		int totaldistan = 0;
		int i;
		for (i=0;i<pathfinder.path.size()-1;i++){
			path.push_back(pathfinder.path[i]);
			cv::line(img,Point(pathfinder.path[i]->pose.x,pathfinder.path[i]->pose.y),Point(pathfinder.path[i+1]->pose.x,pathfinder.path[i+1]->pose.y),Scalar(255,0,0),2);
			totaldistan += getDistance(point(pathfinder.path[i]->pose.x,pathfinder.path[i]->pose.y),point(pathfinder.path[i+1]->pose.x,pathfinder.path[i+1]->pose.y));
		}
		cv::line(img,Point(pathfinder.path[0]->pose.x,pathfinder.path[0]->pose.y),Point(goal.x,goal.y),Scalar(255,0,0),2);
		totaldistan += getDistance(point(pathfinder.path[0]->pose.x,pathfinder.path[0]->pose.y),point(goal.x,goal.y)); //����goal�ľ���
		path.push_back(pathfinder.path[i]);
		std::cout << "total distance is : " << totaldistan << std::endl;	;
		return true;
	}
}





void PRM_RSS_map::showResult(const char * filename) {
	using namespace std;
	using namespace cv;
	string str = "start";
	putText(img, str, Point(start.x, start.y), cv::FONT_HERSHEY_DUPLEX, 2, cv::Scalar(0, 255, 0), 2);
	string str_1 = "goal";
	putText(img, str_1, Point(goal.x, goal.y), cv::FONT_HERSHEY_DUPLEX, 2, cv::Scalar(0, 255, 0), 2);
	namedWindow("map", CV_WINDOW_NORMAL);//���������ڵĴ�С
										 //resizeWindow("map", 640, 480);
	imshow("map", img);
	imwrite(filename, img);
	waitKey();
}