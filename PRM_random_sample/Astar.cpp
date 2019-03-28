#include "Astar.h"

bool Astar::getPath(p_node* start,p_node* goal){
	if (open.size() != 0){
		open.erase(open.begin(),open.end());
	}
	if (closed.size() != 0){
		closed.erase(closed.begin(),closed.end());
	}

	g_node* n = new g_node(start,0,getManDistance(start,goal),NULL);
	open.push_back(n);  // �������� open list

	while (open.size() != 0 ){		//�ظ����¹���
		g_node* current = open[0];
		int delet_flag=0;
		for (int i=1;i<open.size();i++){	//���� open list ������ F ֵ��С�Ľڵ㣬������Ϊ��ǰҪ����Ľڵ�
			if (current->cost > open[i]->cost){
				current = open[i];
				delet_flag=i;
			}		
		}

		if (current->pose == goal){		//���յ���뵽�� open list ��,ֹͣ
			while (current != NULL)	{
				path.push_back(current->pose);
				current=current->father;
			}
			return true;
		}

		open.erase(open.begin() + delet_flag);
		closed.push_back(current); //������ڵ��Ƶ� close list 

		for (int i=0;i<current->pose->neighbors.size();i++){ // ����current������
			int delete_flag_closed=0;
			int delet_flag_open=0;
			p_node* p = current->pose->neighbors[i];  //pΪcurrent�������ڵ�
			int temp_cost_s= current->start_cost + getDistance(current->pose,p);

			g_node* temp=NULL;

			for (int j=0;j<closed.size();j++){  
				if (closed[j]->pose == p){   //������� close list �У�������
					temp = closed[j];
					delete_flag_closed=j;	//��close list�ı�־λ
				}
			}

			if ( (temp != NULL) && temp_cost_s >= temp->start_cost ){   //������� close list �У�������
				continue;
			}
			
			if (temp == NULL){     //��������� close list ��
				for (int j=0;j<open.size();j++){
					if (open[j]->pose == p){
						temp= open[j];
						delet_flag_open=j;
					}
				}

				if (temp == NULL){
					temp= new g_node(p,temp_cost_s,getManDistance(p,goal),current);
					open.push_back(temp);	// ��������� open list �У��������� open list �����Ұѵ�ǰ��������Ϊ���ĸ��ף���¼�÷���� F �� G �� H ֵ
				}

				// ������Ѿ��� open list �У��������·�� ( �����ɵ�ǰ���񵽴������� ) �Ƿ���ã��� G ֵ���ο���
				//��С�� G ֵ��ʾ���Ǹ��õ�·��������������������ĸ�������Ϊ��ǰ���񣬲����¼������� G �� F ֵ�������� open list �ǰ� F ֵ����Ļ����ı���������Ҫ��������
				else if (temp_cost_s < temp->start_cost){
					temp->start_cost=temp_cost_s;
					temp->father=current;
					temp->goal_cost=getManDistance(p,goal);
					temp->cost=temp->start_cost+temp->goal_cost;
					temp->pose=p;
				}

			}else {   //(temp ��= NULL)   // ������� close list �У�������
				temp->start_cost=temp_cost_s;
				temp->father=current;
				temp->goal_cost=getManDistance(p,goal);
				temp->cost=temp->start_cost+temp->goal_cost;
				temp->pose=p;
				closed.erase(closed.begin()+delete_flag_closed);		//��close list�ı�־λ��delete_flag_closed
				open.push_back(temp);
			}	

		}  //����current�����ڣ�����
	}// open_listΪ��
	return false;
}