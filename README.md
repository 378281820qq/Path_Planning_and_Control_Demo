# Path_Planning_and_Control_Demo

Reference:
https://github.com/AtsushiSakai/PythonRobotics

Control folder:

1.move_to_pose_PID.py 
  Using PID controller to move the vehicle to a goal position.The PID controller controls both the velocity and steering angle
 
2.pure_pursuit.py
  预瞄法，找到路径上的在当前位置的前面一点，计算出当前的需要的方向盘转角，PI controller
 
3.Frenet_optimal_trajectroy folder:

    1. frenet_optimal_trajectory: convert the x-y coordinates to s-d coordinates
    
                                  sampling in road_width & sampling in time period & sampling in longitudinal velocity
                                  
                                  get the min cost sampling set      
                                  
                                  from the sampling set to create the 3rd or 4th polynomial curve which has a min jerk value
                                  
                                  using the polynomial curve and sampling time period to caluculate the next-time s,d,ds/dt, dd/dt
                                  
                                  convert s,d,ds/dt, dd/dt to x,y,dx/dt,dy/dt
                                  
                                  send the next-time x,y,dx/dt,dy/dtx,y,dx/dt,dy/dt to low-level controller  
                                  
    2. cubic_spline_planner: from points to generate a 3rd curve  
 
 
 
 4.MPC_vehicle_control:  control a car's velcity and steering angle to reach a goal point
 
                         MPC can be formed as a standard formate
    
                         The solver using decent gradient method to choose control inputs and solve the MPC problem   
 
 
 

5.A_star.py, A_star.cpp: g=f+h

6.Dynamic_window_approach: another form of sampling controller,sampling in velocity and acceleration
                            
                            (1)在控制空间中离散采样多组速度dx,dy,dtheta
                            (2)对每个采样的速度向量dx,dy,dtheta，模拟机器人在这种速度下，预测前进一个或者多个采样时间段机器的行走轨迹以及会发生什么。
                            (3)对每个前进预测进行分析打分，choose the min cost path

                             

7.RRT: the sampling method
    1.初始化起始点。比如设置机器人所在的位置为初始点；

    2.随机生成目标点，遍历T，如果通过T能到达目标点，则路径搜索成功，扩展结束；否则继续扩展T；

    3.挑选随机点到目标点最近的一个为Xnear;

    4.沿着Xrand到Xnear的方向生长一段距离，生成一个新的节点Xnew;

    5.判断Xnew进行碰撞检测，如果状态非法，则本次生长结束；否则，将新的状态添加到T；

    6.返回树结构。   


    function BuildRRT(qinit, K, Δq)
    T.init(qinit)
    for k = 1 to K
        qrand = Sample()  -- chooses a random configuration
        qnearest = Nearest(T, qrand) -- selects the node in the RRT tree that is closest to qrand
        if  Distance(qnearest, qgoal) < Threshold then
            return true
        qnew = Extend(qnearest, qrand, Δq)  -- moving from qnearest an incremental distance in the direction of qrand
        if qnew ≠ NULL then
            T.AddNode(qnew)
    return false


    function Sample() -- Alternatively,one could replace Sample with SampleFree(by using a collision detection algorithm to reject samples in C_obstacle
        p = Random(0, 1.0)
        if 0 < p < Prob then
            return qgoal
        elseif Prob < p < 1.0 then
            return RandomNode()
       


8.PRM(Probabilistic Roadmaps) 是一种基于图搜索的方法，一共分为两个步骤：学习阶段， 查询阶段     
           它将连续空间转换成离散空间，再利用A*等搜索算法在路线图上寻找路径，以提高搜索效率
