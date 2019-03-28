//http://blog.tk-xiong.com/archives/671
//https://blog.csdn.net/lbaihao/article/details/77427411
// https://blog.csdn.net/hitwhylz/article/details/23089415
//https://blog.csdn.net/DinnerHowe/article/details/80267062
/*
The VS demo:
opencv 2.4.13
VS2015
win10 64
*/


#include "PRM_RSS.h"
#include <time.h>
#include <stdio.h>
#define MI 1

int main()
{
	PRM_RSS_map map("intel_binary.jpg",point(652,1158),point(498,312));
	//map.getPath();
	//map.showResult("PRM随机撒点1.jpg");


	
	bool Path_is_availble = map.getPath();
	int i = 1;
	while (!Path_is_availble) {
		PRM_RSS_map map("intel_binary.jpg", point(652, 1158), point(498, 312));
		Path_is_availble = map.getPath();
		i++;
	}
	map.showResult("PRM随机撒点1.jpg");
	std::cout << i << std::endl;



	return 0;
}