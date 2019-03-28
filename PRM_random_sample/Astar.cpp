#include "Astar.h"

bool Astar::getPath(p_node* start,p_node* goal){
	if (open.size() != 0){
		open.erase(open.begin(),open.end());
	}
	if (closed.size() != 0){
		closed.erase(closed.begin(),closed.end());
	}

	g_node* n = new g_node(start,0,getManDistance(start,goal),NULL);
	open.push_back(n);  // 把起点加入 open list

	while (open.size() != 0 ){		//重复如下过程
		g_node* current = open[0];
		int delet_flag=0;
		for (int i=1;i<open.size();i++){	//遍历 open list ，查找 F 值最小的节点，把它作为当前要处理的节点
			if (current->cost > open[i]->cost){
				current = open[i];
				delet_flag=i;
			}		
		}

		if (current->pose == goal){		//把终点加入到了 open list 中,停止
			while (current != NULL)	{
				path.push_back(current->pose);
				current=current->father;
			}
			return true;
		}

		open.erase(open.begin() + delet_flag);
		closed.push_back(current); //把这个节点移到 close list 

		for (int i=0;i<current->pose->neighbors.size();i++){ // 搜索current的相邻
			int delete_flag_closed=0;
			int delet_flag_open=0;
			p_node* p = current->pose->neighbors[i];  //p为current的相连节点
			int temp_cost_s= current->start_cost + getDistance(current->pose,p);

			g_node* temp=NULL;

			for (int j=0;j<closed.size();j++){  
				if (closed[j]->pose == p){   //如果它在 close list 中，忽略它
					temp = closed[j];
					delete_flag_closed=j;	//在close list的标志位
				}
			}

			if ( (temp != NULL) && temp_cost_s >= temp->start_cost ){   //如果它在 close list 中，忽略它
				continue;
			}
			
			if (temp == NULL){     //如果它不在 close list 中
				for (int j=0;j<open.size();j++){
					if (open[j]->pose == p){
						temp= open[j];
						delet_flag_open=j;
					}
				}

				if (temp == NULL){
					temp= new g_node(p,temp_cost_s,getManDistance(p,goal),current);
					open.push_back(temp);	// 如果它不在 open list 中，把它加入 open list ，并且把当前方格设置为它的父亲，记录该方格的 F ， G 和 H 值
				}

				// 如果它已经在 open list 中，检查这条路径 ( 即经由当前方格到达它那里 ) 是否更好，用 G 值作参考。
				//更小的 G 值表示这是更好的路径。如果是这样，把它的父亲设置为当前方格，并重新计算它的 G 和 F 值。如果你的 open list 是按 F 值排序的话，改变后你可能需要重新排序。
				else if (temp_cost_s < temp->start_cost){
					temp->start_cost=temp_cost_s;
					temp->father=current;
					temp->goal_cost=getManDistance(p,goal);
					temp->cost=temp->start_cost+temp->goal_cost;
					temp->pose=p;
				}

			}else {   //(temp ！= NULL)   // 如果它在 close list 中，忽略它
				temp->start_cost=temp_cost_s;
				temp->father=current;
				temp->goal_cost=getManDistance(p,goal);
				temp->cost=temp->start_cost+temp->goal_cost;
				temp->pose=p;
				closed.erase(closed.begin()+delete_flag_closed);		//在close list的标志位：delete_flag_closed
				open.push_back(temp);
			}	

		}  //搜索current的相邻，结束
	}// open_list为空
	return false;
}