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

                         算法组成：预测模型、反馈校正、参考轨迹、滚动优化

                         MPC 基本原理:

首先指出，模型预测控制以计算机为实现手段，因此 MPC 采用采样（或离散）的控制算法。通常来说，不管该算法有何种表现方式，都遵循以下基本原理：  



 (1) 预测模型MPC是以模型为基础的控制算法，此模型称作预测模型。预测模型的作用是利用对象未来输入和历史信息预测未来输出。

   预测模型的形式可以是状态方程、传递函数，甚至非参数模型阶跃响应和脉冲响应可作为线性稳定对象的预测模型。
   
 (2) 反馈校正 MPC 是闭环控制算法，经过优化作用确定一系列未来控制作用，考虑环境干扰、模型失陪会导致控制作用对期望状态的较大偏差，一般不将控制作用全部实施，

      只利用本时刻的控制作用。到下一采样时刻，通过检测对象实际输出修正基于模型的预测作用，之后开始新时刻的优化，反馈校正都把优化作用建立在系统实际基础上。

      因此，MPC 中优化不仅基于模型，也利用反馈信息，因而构成了闭环优化。

   
 (3) 滚动优化 MPC 是一种通过计算某一性能指标的极值求取未来控制作用的优化控制算法。性能指标的选取通常取为在未来采样点上跟踪某一期望轨迹方差最小：

      min J =sum{i=1}{p}[x(k+i|k)]{T}Q[x(k+i|k)-x_r(k+i|k)]

    其中，x 表示优化值，x_r 表示参考轨迹，Q 表示权系数，表示对跟踪误差的抑制，P 表示优化时域，k 表示第 k 个采样时刻。

        MPC 中的优化形式与一般的离散优化相比有很大的不同，不同点主要在于 MPC 优化方式是一种有限时域滚动形式的优化。

        在采样时刻 k，性能指标时间区域只涉及到从 k 时刻到未来(k+P) 的有限时刻，到下一采样时刻(k+1) ，优化时段随之向前滚动.所以，MPC 不采用全局

        的性能指标，在每个采样时刻对应一个优化性能指标。不同时刻优化性能指标具有相同的形式，但包含不同的时间区域。MPC 中优化是不断反复在线进行，

        传统最优控制优化是一次离线进行，这是两者的最大不同点。
         
        从上可以看出，MPC 鲜明特征是基于模型、滚动优化且包含反馈校正的计算机控制算法。这种控制算法能顾及到模型误差、环境干扰不确定影响并及时校正,比一次优化更能适应实际过程，有更强鲁棒性。
        

  CVXPY是一种可以内置于Python中的模型编程语言，解决凸优化问题。它可以自动转化问题为标准形式，调用解法器，解包结果集
 

5.A_star.py, A_star.cpp: g=f+h


        1、将开始点记录为当前点P

        2、将当前点P放入封闭列表

        3、搜寻点P所有邻近点，假如某邻近点既没有在开放列表或封闭列表里面，则计算出该邻近点的F值，并设父节点为P，然后将其放入开放列表

        4、判断开放列表是否已经空了，如果没有说明在达到结束点前已经找完了所有可能的路径点，寻路失败，算法结束；否则继续。

        5、从开放列表拿出一个F值最小的点，作为寻路路径的下一步。

        6、判断该点是否为结束点，如果是，则寻路成功，算法结束；否则继续。

        7、将该点设为当前点P，跳回步骤3。



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
