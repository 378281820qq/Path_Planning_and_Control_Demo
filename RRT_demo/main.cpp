//https://blog.csdn.net/lbaihao/article/details/77427411
//http://blog.tk-xiong.com/archives/671
//https://github.com/gouxiangchen/RRT-path-planning
//https://blog.csdn.net/gpeng832/article/details/71249198
/*
The VS demo:
opencv 2.4.13
VS2015
win10 64
*/

#include <iostream>
#include "RRT.h"
#include "map_pgm.h"




int main(){

	RRT	map("intel_binary.jpg",point(652,1158),point(498,312));


	map.createTree();

	if (map.getPath()) 	{ 
		map.showResult("RRT.jpg"); 
	}
	return 0;
}